#include "ilw.hpp"
#include <ftl/utility/matrix_conversion.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/cuda/points.hpp>
#include <loguru.hpp>

#include "ilw_cuda.hpp"

using ftl::ILW;
using ftl::detail::ILWData;
using ftl::codecs::Channel;
using ftl::codecs::Channels;
using ftl::rgbd::Format;
using cv::cuda::GpuMat;

// TODO: Put in common place
static Eigen::Affine3d create_rotation_matrix(float ax, float ay, float az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * rx * ry;
}

ILW::ILW(nlohmann::json &config) : ftl::Configurable(config) {
    enabled_ = value("ilw_align", true);
    iterations_ = value("iterations", 4);
    motion_rate_ = value("motion_rate", 0.8f);
    motion_window_ = value("motion_window", 3);
    use_lab_ = value("use_Lab", false);
    params_.colour_smooth = value("colour_smooth", 50.0f);
    params_.spatial_smooth = value("spatial_smooth", 0.04f);
    params_.cost_ratio = value("cost_ratio", 0.2f);
	params_.cost_threshold = value("cost_threshold", 1.0f);
	discon_mask_ = value("discontinuity_mask",2);
	fill_depth_ = value("fill_depth", false);

    on("fill_depth", [this](const ftl::config::Event &e) {
        fill_depth_ = value("fill_depth", false);
    });

	on("ilw_align", [this](const ftl::config::Event &e) {
        enabled_ = value("ilw_align", true);
    });

    on("iterations", [this](const ftl::config::Event &e) {
        iterations_ = value("iterations", 4);
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
        params_.cost_ratio = value("cost_ratio", 0.2f);
    });

	on("cost_threshold", [this](const ftl::config::Event &e) {
        params_.cost_threshold = value("cost_threshold", 1.0f);
    });

    params_.flags = 0;
    if (value("ignore_bad", false)) params_.flags |= ftl::cuda::kILWFlag_IgnoreBad;
    if (value("ignore_bad_colour", false)) params_.flags |= ftl::cuda::kILWFlag_SkipBadColour;
    if (value("restrict_z", true)) params_.flags |= ftl::cuda::kILWFlag_RestrictZ;
    if (value("colour_confidence_only", false)) params_.flags |= ftl::cuda::kILWFlag_ColourConfidenceOnly;

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

    on("colour_confidence_only", [this](const ftl::config::Event &e) {
        if (value("colour_confidence_only", false)) params_.flags |= ftl::cuda::kILWFlag_ColourConfidenceOnly;
        else params_.flags &= ~ftl::cuda::kILWFlag_ColourConfidenceOnly;
    });

	if (config["clipping"].is_object()) {
		auto &c = config["clipping"];
		float rx = c.value("pitch", 0.0f);
		float ry = c.value("yaw", 0.0f);
		float rz = c.value("roll", 0.0f);
		float x = c.value("x", 0.0f);
		float y = c.value("y", 0.0f);
		float z = c.value("z", 0.0f);
		float width = c.value("width", 1.0f);
		float height = c.value("height", 1.0f);
		float depth = c.value("depth", 1.0f);

		Eigen::Affine3f r = create_rotation_matrix(rx, ry, rz).cast<float>();
		Eigen::Translation3f trans(Eigen::Vector3f(x,y,z));
		Eigen::Affine3f t(trans);

		clip_.origin = MatrixConversion::toCUDA(r.matrix() * t.matrix());
		clip_.size = make_float3(width, height, depth);
		clipping_ = value("clipping_enabled", true);
	} else {
		clipping_ = false;
	}

	on("clipping_enabled", [this](const ftl::config::Event &e) {
		clipping_ = value("clipping_enabled", true);
	});

	cudaSafeCall(cudaStreamCreate(&stream_));
}

ILW::~ILW() {

}

bool ILW::process(ftl::rgbd::FrameSet &fs) {
    if (!enabled_) return false;

	//fs.upload(Channel::Colour + Channel::Depth, stream_);
    _phase0(fs, stream_);

	params_.range = value("search_range", 0.05f);
    params_.fill_match = value("fill_match", 50.0f);
    params_.fill_threshold = value("fill_threshold", 0.0f);
	params_.match_threshold = value("match_threshold", 0.3f);

    for (int i=0; i<iterations_; ++i) {
        _phase1(fs, value("cost_function",3), stream_);
        //for (int j=0; j<3; ++j) {
            _phase2(fs, motion_rate_, stream_);
        //}

		params_.range *= value("search_reduce", 0.9f);
		// TODO: Break if no time left

		//cudaSafeCall(cudaStreamSynchronize(stream_));
    }

	for (size_t i=0; i<fs.frames.size(); ++i) {
		auto &f = fs.frames[i];
		auto *s = fs.sources[i];
			
        auto &t = f.createTexture<float4>(Channel::Points, Format<float4>(f.get<GpuMat>(Channel::Colour).size()));
        auto pose = MatrixConversion::toCUDA(s->getPose().cast<float>()); //.inverse());
        ftl::cuda::point_cloud(t, f.createTexture<float>(Channel::Depth), s->parameters(), pose, discon_mask_, stream_);
    }

	cudaSafeCall(cudaStreamSynchronize(stream_));
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
			
        //auto &t = f.createTexture<float4>(Channel::Points, Format<float4>(f.get<GpuMat>(Channel::Colour).size()));
        auto pose = MatrixConversion::toCUDA(s->getPose().cast<float>()); //.inverse());
        //ftl::cuda::point_cloud(t, f.createTexture<float>(Channel::Depth), s->parameters(), pose, discon_mask_, stream);

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

		// Clip first?
		if (clipping_) {
			auto clip = clip_;
			clip.origin = clip.origin * pose;
			ftl::cuda::clipping(f.createTexture<float>(Channel::Depth), s->parameters(), clip, stream);
		}

        f.createTexture<float>(Channel::Depth2, Format<float>(f.get<GpuMat>(Channel::Colour).size()));
        f.createTexture<float>(Channel::Confidence, Format<float>(f.get<GpuMat>(Channel::Colour).size()));
		f.createTexture<int>(Channel::Mask, Format<int>(f.get<GpuMat>(Channel::Colour).size()));
        f.createTexture<uchar4>(Channel::Colour);
		f.createTexture<float>(Channel::Depth);

		f.get<GpuMat>(Channel::Mask).setTo(cv::Scalar(0));
    }

	//cudaSafeCall(cudaStreamSynchronize(stream_));

    return true;
}

bool ILW::_phase1(ftl::rgbd::FrameSet &fs, int win, cudaStream_t stream) {
    // Run correspondence kernel to create an energy vector
    cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

    // Find discontinuity mask
    for (size_t i=0; i<fs.frames.size(); ++i) {
        auto &f = fs.frames[i];
        auto s = fs.sources[i];

        ftl::cuda::discontinuity(
            f.getTexture<int>(Channel::Mask),
            f.getTexture<float>(Channel::Depth),
            s->parameters(),
            discon_mask_,
            stream
        );
    }

	// First do any preprocessing
	if (fill_depth_) {
		for (size_t i=0; i<fs.frames.size(); ++i) {
			auto &f = fs.frames[i];
            auto s = fs.sources[i];

			ftl::cuda::preprocess_depth(
				f.getTexture<float>(Channel::Depth),
				f.getTexture<float>(Channel::Depth2),
				f.getTexture<uchar4>(Channel::Colour),
				f.getTexture<int>(Channel::Mask),
				s->parameters(),
				params_,
				stream
			);

			//cv::cuda::swap(f.get<GpuMat>(Channel::Depth),f.get<GpuMat>(Channel::Depth2)); 
			f.swapChannels(Channel::Depth, Channel::Depth2);
		}
	}

	//cudaSafeCall(cudaStreamSynchronize(stream_));

	// For each camera combination
    for (size_t i=0; i<fs.frames.size(); ++i) {
        auto &f1 = fs.frames[i];
        f1.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0.0f), cvstream);
		f1.get<GpuMat>(Channel::Confidence).setTo(cv::Scalar(0.0f), cvstream);

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
            ftl::cuda::correspondence(
                f1.getTexture<float>(Channel::Depth),
                f2.getTexture<float>(Channel::Depth),
                f1.getTexture<uchar4>(Channel::Colour),
                f2.getTexture<uchar4>(Channel::Colour),
                // TODO: Add normals and other things...
                f1.getTexture<float>(Channel::Depth2),
                f1.getTexture<float>(Channel::Confidence),
				f1.getTexture<int>(Channel::Mask),
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

	//cudaSafeCall(cudaStreamSynchronize(stream_));

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
             f.getTexture<float>(Channel::Depth),
             f.getTexture<float>(Channel::Depth2),
			 f.getTexture<float>(Channel::Confidence),
             fs.sources[i]->parameters(),
             pose,
			 params_,
             rate,
             motion_window_,
             stream
        );

        if (f.hasChannel(Channel::Mask)) {
			ftl::cuda::mask_filter(f.getTexture<float>(Channel::Depth),
				f.getTexture<int>(Channel::Mask), stream_);
		}
    }

    return true;
}
