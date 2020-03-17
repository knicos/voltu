#include "ftl/operators/disparity.hpp"
#include <ftl/operators/cuda/disparity.hpp>

using ftl::operators::DisparityToDepth;
using ftl::codecs::Channel;

using cv::cuda::GpuMat;

bool DisparityToDepth::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out,
							cudaStream_t stream) {
	
	if (!in.hasChannel(Channel::Disparity)) {
		throw FTL_Error("Missing disparity before convert to depth");
	}

	const GpuMat &disp = in.get<GpuMat>(Channel::Disparity);
	const auto params = in.getLeftCamera().scaled(disp.cols, disp.rows);

	GpuMat &depth = out.create<GpuMat>(Channel::Depth);
	depth.create(disp.size(), CV_32FC1);

	ftl::cuda::disparity_to_depth(disp, depth, params, stream);
	return true;
}