#include "ftl/operators/disparity.hpp"
#include <ftl/operators/cuda/disparity.hpp>

using ftl::operators::DisparityToDepth;
using ftl::codecs::Channel;
using ftl::operators::Buffer;

using cv::cuda::GpuMat;

bool DisparityToDepth::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out,
							cudaStream_t stream) {

	if (!graph()->hasBuffer(Buffer::Disparity, in.source())) {
		throw FTL_Error("Missing disparity before convert to depth");
	}

	const GpuMat &disp = graph()->getBuffer(Buffer::Disparity, in.source());
	const auto params = in.getLeftCamera().scaled(disp.cols, disp.rows);

	GpuMat &depth = out.create<GpuMat>(Channel::Depth);
	depth.create(disp.size(), CV_32FC1);

	ftl::cuda::disparity_to_depth<short, float>(disp, depth, params, 1.0f/16.0f, stream);
	return true;
}