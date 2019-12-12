#include "ftl/operators/disparity.hpp"
#include "disparity/cuda.hpp"

using ftl::operators::DisparityToDepth;
using ftl::codecs::Channel;

using cv::cuda::GpuMat;

bool DisparityToDepth::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out,
							 ftl::rgbd::Source *src, cudaStream_t stream) {
	
	if (!in.hasChannel(Channel::Disparity)) { return false;  }

	const auto params = src->parameters();
	const GpuMat &disp = in.get<GpuMat>(Channel::Disparity);

	GpuMat &depth = out.create<GpuMat>(Channel::Depth);
	depth.create(disp.size(), CV_32FC1);

	ftl::cuda::disparity_to_depth(disp, depth, params, stream);
	return true;
}