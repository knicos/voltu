#include <ftl/operators/mask.hpp>
#include "mask_cuda.hpp"

using ftl::operators::DiscontinuityMask;
using ftl::operators::CullDiscontinuity;
using ftl::codecs::Channel;
using ftl::rgbd::Format;

DiscontinuityMask::DiscontinuityMask(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

DiscontinuityMask::~DiscontinuityMask() {

}

bool DiscontinuityMask::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	int radius = config()->value("radius", 2);
	float threshold = config()->value("threshold", 0.1f);

	ftl::cuda::discontinuity(
		out.createTexture<int>(Channel::Mask, ftl::rgbd::Format<int>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
		in.createTexture<uchar4>(Channel::Support1),
		in.createTexture<float>(Channel::Depth),
		in.get<cv::cuda::GpuMat>(Channel::Depth).size(),
		s->parameters().minDepth, s->parameters().maxDepth,
		radius, threshold, stream
	);

	return true;
}



CullDiscontinuity::CullDiscontinuity(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

CullDiscontinuity::~CullDiscontinuity() {

}

bool CullDiscontinuity::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	ftl::cuda::cull_discontinuity(
		in.createTexture<int>(Channel::Mask),
		out.createTexture<float>(Channel::Depth),
		stream
	);

	return true;
}