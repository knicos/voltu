#include "ilw.hpp"
#include <ftl/utility/matrix_conversion.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/cuda/points.hpp>

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

    for (int i=0; i<2; ++i) {
        _phase1(fs);
        for (int j=0; j<3; ++j) {
            _phase2(fs);
        }

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
        ftl::cuda::point_cloud(t, f.createTexture<float>(Channel::Depth), s->parameters(), pose, stream);
    }

	// Upload camera data?
    return true;
}

bool ILW::_phase1(ftl::rgbd::FrameSet &fs) {
    // Run correspondence kernel to find points

	// For each camera combination

    return true;
}

bool ILW::_phase2(ftl::rgbd::FrameSet &fs) {
    // Run energies and motion kernel

    return true;
}
