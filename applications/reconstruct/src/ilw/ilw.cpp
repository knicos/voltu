#include "ilw.hpp"
#include <ftl/utility/matrix_conversion.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/cuda/points.hpp>
#include <loguru.hpp>

#include "ilw_cuda.hpp"

using ftl::ILW;
using ftl::detail::ILWData;
using ftl::rgbd::Channel;
using ftl::rgbd::Channels;
using ftl::rgbd::Format;
using cv::cuda::GpuMat;

ILW::ILW(nlohmann::json &config) : ftl::Configurable(config) {
    enabled_ = value("ilw_align", true);
    iterations_ = value("iterations", 1);
    motion_rate_ = value("motion_rate", 0.4f);
    motion_window_ = value("motion_window", 3);
    use_lab_ = value("use_Lab", false);
    params_.colour_smooth = value("colour_smooth", 50.0f);
    params_.spatial_smooth = value("spatial_smooth", 0.04f);
    params_.cost_ratio = value("cost_ratio", 0.75f);
	discon_mask_ = value("discontinuity_mask",2);

    on("ilw_align", [this](const ftl::config::Event &e) {
        enabled_ = value("ilw_align", true);
    });

    on("iterations", [this](const ftl::config::Event &e) {
        iterations_ = value("iterations", 1);
    });

    on("motion_rate", [this](const ftl::config::Event &e) {
        motion_rate_ = value("motion_rate", 0.4f);
    });

    on("motion_window", [this](const ftl::config::Event &e) {
        motion_window_ = value("motion_window", 3);
    });

	on("discontinuity_mask", [this](const ftl::config::Event &e) {
        discon_mask_ = value("discontinuity_mask", 2);
    });

    on("use_Lab", [this](const ftl::config::Event &e) {
        use_lab_ = value("use_Lab", false);
    });

    on("colour_smooth", [this](const ftl::config::Event &e) {
        params_.colour_smooth = value("colour_smooth", 50.0f);
    });

    on("spatial_smooth", [this](const ftl::config::Event &e) {
        params_.spatial_smooth = value("spatial_smooth", 0.04f);
    });

    on("cost_ratio", [this](const ftl::config::Event &e) {
        params_.cost_ratio = value("cost_ratio", 75.0f);
    });

    params_.flags = 0;
    if (value("ignore_bad", false)) params_.flags |= ftl::cuda::kILWFlag_IgnoreBad;
    if (value("ignore_bad_colour", false)) params_.flags |= ftl::cuda::kILWFlag_SkipBadColour;
    if (value("restrict_z", true)) params_.flags |= ftl::cuda::kILWFlag_RestrictZ;

    on("ignore_bad", [this](const ftl::config::Event &e) {
        if (value("ignore_bad", false)) params_.flags |= ftl::cuda::kILWFlag_IgnoreBad;
        else params_.flags &= ~ftl::cuda::kILWFlag_IgnoreBad;
    });

    on("ignore_bad_colour", [this](const ftl::config::Event &e) {
        if (value("ignore_bad_colour", false)) params_.flags |= ftl::cuda::kILWFlag_SkipBadColour;
        else params_.flags &= ~ftl::cuda::kILWFlag_SkipBadColour;
    });

    on("restrict_z", [this](const ftl::config::Event &e) {
        if (value("restrict_z", false)) params_.flags |= ftl::cuda::kILWFlag_RestrictZ;
        else params_.flags &= ~ftl::cuda::kILWFlag_RestrictZ;
    });
}

ILW::~ILW() {

}

bool ILW::process(ftl::rgbd::FrameSet &fs, cudaStream_t stream) {
    if (!enabled_) return false;

    _phase0(fs, stream);

    for (int i=0; i<iterations_; ++i) {
        int win;
        switch (i) {
        case 0: win = 17; break;
        case 1: win = 9; break;
        default: win = 5; break;
        }
        _phase1(fs, win, stream);
        //for (int j=0; j<3; ++j) {
            _phase2(fs, motion_rate_, stream);
        //}

		// TODO: Break if no time left
    }

    return true;
}

bool ILW::_phase0(ftl::rgbd::FrameSet &fs, cudaStream_t stream) {
    // Make points channel...
    for (size_t i=0; i<fs.frames.size(); ++i) {
		auto &f = fs.frames[i];
		auto *s = fs.sources[i];

		if (f.empty(Channel::Depth + Channel::Colour)) {
			LOG(ERROR) << "Missing required channel";
			continue;
		}
			
        auto &t = f.createTexture<float4>(Channel::Points, Format<float4>(f.get<GpuMat>(Channel::Colour).size()));
        auto pose = MatrixConversion::toCUDA(s->getPose().cast<float>()); //.inverse());
        ftl::cuda::point_cloud(t, f.createTexture<float>(Channel::Depth), s->parameters(), pose, discon_mask_, stream);

        // TODO: Create energy vector texture and clear it
        // Create energy and clear it

        // Convert colour from BGR to BGRA if needed
		if (f.get<GpuMat>(Channel::Colour).type() == CV_8UC3) {
            cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
			// Convert to 4 channel colour
			auto &col = f.get<GpuMat>(Channel::Colour);
			GpuMat tmp(col.size(), CV_8UC4);
			cv::cuda::swap(col, tmp);
            if (use_lab_) cv::cuda::cvtColor(tmp,tmp, cv::COLOR_BGR2Lab, 0, cvstream);
			cv::cuda::cvtColor(tmp,col, cv::COLOR_BGR2BGRA, 0, cvstream);
		}

        f.createTexture<float4>(Channel::EnergyVector, Format<float4>(f.get<GpuMat>(Channel::Colour).size()));
        f.createTexture<float>(Channel::Energy, Format<float>(f.get<GpuMat>(Channel::Colour).size()));
        f.createTexture<uchar4>(Channel::Colour);
    }

    return true;
}

bool ILW::_phase1(ftl::rgbd::FrameSet &fs, int win, cudaStream_t stream) {
    // Run correspondence kernel to create an energy vector
    cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	// For each camera combination
    for (size_t i=0; i<fs.frames.size(); ++i) {
        auto &f1 = fs.frames[i];
        f1.get<GpuMat>(Channel::EnergyVector).setTo(cv::Scalar(0.0f,0.0f,0.0f,0.0f), cvstream);
		f1.get<GpuMat>(Channel::Energy).setTo(cv::Scalar(0.0f), cvstream);

		Eigen::Vector4d d1(0.0, 0.0, 1.0, 0.0);
		d1 = fs.sources[i]->getPose() * d1;

        for (size_t j=0; j<fs.frames.size(); ++j) {
            if (i == j) continue;

            //LOG(INFO) << "Running phase1";

            auto &f2 = fs.frames[j];
            auto s1 = fs.sources[i];
            auto s2 = fs.sources[j];

			// Are cameras facing similar enough direction?
			Eigen::Vector4d d2(0.0, 0.0, 1.0, 0.0);
			d2 = fs.sources[j]->getPose() * d2;
			// No, so skip this combination
			if (d1.dot(d2) <= 0.0) continue;

            auto pose1 = MatrixConversion::toCUDA(s1->getPose().cast<float>());
            auto pose1_inv = MatrixConversion::toCUDA(s1->getPose().cast<float>().inverse());
            auto pose2 = MatrixConversion::toCUDA(s2->getPose().cast<float>().inverse());

            try {
            //Calculate energy vector to best correspondence
            ftl::cuda::correspondence_energy_vector(
                f1.getTexture<float4>(Channel::Points),
                f2.getTexture<float4>(Channel::Points),
                f1.getTexture<uchar4>(Channel::Colour),
                f2.getTexture<uchar4>(Channel::Colour),
                // TODO: Add normals and other things...
                f1.getTexture<float4>(Channel::EnergyVector),
                f1.getTexture<float>(Channel::Energy),
                pose1,
                pose1_inv,
                pose2,
                s1->parameters(),
                s2->parameters(),
                params_,
                win,
                stream
            );
            } catch (ftl::exception &e) {
                LOG(ERROR) << "Exception in correspondence: " << e.what();
            }

            //LOG(INFO) << "Correspondences done... " << i;
        }
    }

    return true;
}

bool ILW::_phase2(ftl::rgbd::FrameSet &fs, float rate, cudaStream_t stream) {
    // Run energies and motion kernel

	// Smooth vectors across a window and iteratively
	// strongly disagreeing vectors should cancel out
	// A weak vector is overriden by a stronger one.

    for (size_t i=0; i<fs.frames.size(); ++i) {
        auto &f = fs.frames[i];

        auto pose = MatrixConversion::toCUDA(fs.sources[i]->getPose().cast<float>()); //.inverse());

        ftl::cuda::move_points(
             f.getTexture<float4>(Channel::Points),
             f.getTexture<float4>(Channel::EnergyVector),
             fs.sources[i]->parameters(),
             pose,
             rate,
             motion_window_,
             stream
        );
    }

    return true;
}
