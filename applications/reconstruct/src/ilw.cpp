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

}

ILW::~ILW() {

}

bool ILW::process(ftl::rgbd::FrameSet &fs, cudaStream_t stream) {
    _phase0(fs, stream);

    //for (int i=0; i<2; ++i) {
        _phase1(fs, stream);
        //for (int j=0; j<3; ++j) {
        //    _phase2(fs);
        //}

		// TODO: Break if no time left
    //}

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
        ftl::cuda::point_cloud(t, f.createTexture<float>(Channel::Depth), s->parameters(), pose, stream);

        // TODO: Create energy vector texture and clear it
        // Create energy and clear it

        // Convert colour from BGR to BGRA if needed
		if (f.get<GpuMat>(Channel::Colour).type() == CV_8UC3) {
			// Convert to 4 channel colour
			auto &col = f.get<GpuMat>(Channel::Colour);
			GpuMat tmp(col.size(), CV_8UC4);
			cv::cuda::swap(col, tmp);
			cv::cuda::cvtColor(tmp,col, cv::COLOR_BGR2BGRA);
		}

        f.createTexture<float4>(Channel::EnergyVector, Format<float4>(f.get<GpuMat>(Channel::Colour).size()));
        f.createTexture<float>(Channel::Energy, Format<float>(f.get<GpuMat>(Channel::Colour).size()));
        f.createTexture<uchar4>(Channel::Colour);

		cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

		f.get<GpuMat>(Channel::EnergyVector).setTo(cv::Scalar(0.0f,0.0f,0.0f,0.0f), cvstream);
		f.get<GpuMat>(Channel::Energy).setTo(cv::Scalar(0.0f), cvstream);
    }

    return true;
}

bool ILW::_phase1(ftl::rgbd::FrameSet &fs, cudaStream_t stream) {
    // Run correspondence kernel to create an energy vector

	// For each camera combination
    for (size_t i=0; i<fs.frames.size(); ++i) {
        for (size_t j=0; j<fs.frames.size(); ++j) {
            if (i == j) continue;

            LOG(INFO) << "Running phase1";

            auto &f1 = fs.frames[i];
            auto &f2 = fs.frames[j];
            //auto s1 = fs.frames[i];
            auto s2 = fs.sources[j];

            auto pose = MatrixConversion::toCUDA(s2->getPose().cast<float>().inverse());

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
                pose,
                s2->parameters(),
                stream
            );
            } catch (ftl::exception &e) {
                LOG(ERROR) << "Exception in correspondence: " << e.what();
            }

            LOG(INFO) << "Correspondences done... " << i;
        }
    }

    return true;
}

bool ILW::_phase2(ftl::rgbd::FrameSet &fs) {
    // Run energies and motion kernel

    return true;
}
