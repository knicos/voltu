#include <ftl/operators/mask.hpp>
#include <ftl/operators/mask_cuda.hpp>

using ftl::operators::DiscontinuityMask;
using ftl::operators::BorderMask;
using ftl::operators::CullDiscontinuity;
using ftl::codecs::Channel;
using ftl::rgbd::Format;

DiscontinuityMask::DiscontinuityMask(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

DiscontinuityMask::~DiscontinuityMask() {

}

bool DiscontinuityMask::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (in.hasChannel(Channel::Mask)) return true;
	
	//int radius = config()->value("radius", 2);
	float disconPixels = config()->value("discon_pixels", 15.0f);
	float depthCoef = (1.0f / in.getLeft().fx) * disconPixels;
	float disconThresh = config()->value("discon_thresh", 0.8f);
	float noiseThresh = config()->value("noise_thresh", 0.8f);
	float areaMax = config()->value("area_max", 26.0f);  // Cross support radius squared + 1

	if (!in.hasChannel(Channel::Depth) || !in.hasChannel(Channel::Support1)) return false;

	/*ftl::cuda::discontinuity(
		out.createTexture<uint8_t>(Channel::Mask, ftl::rgbd::Format<uint8_t>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
		in.createTexture<uchar4>(Channel::Support1),
		in.createTexture<float>(Channel::Depth),
		in.get<cv::cuda::GpuMat>(Channel::Depth).size(),
		in.getLeftCamera().minDepth, in.getLeftCamera().maxDepth,
		radius, depthCoef, stream
	);*/

	ftl::cuda::discontinuity(
		out.createTexture<uint8_t>(Channel::Mask, ftl::rgbd::Format<uint8_t>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
		in.createTexture<uchar4>(Channel::Support1),
		in.createTexture<float>(Channel::Depth),
		in.get<cv::cuda::GpuMat>(Channel::Depth).size(),
		in.getLeftCamera().minDepth, in.getLeftCamera().maxDepth,
		depthCoef, disconThresh, noiseThresh, areaMax, stream
	);

	return true;
}



BorderMask::BorderMask(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

BorderMask::~BorderMask() {

}

bool BorderMask::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	int leftm = config()->value("left", 100);
	int rightm = config()->value("right", 5);
	int topm = config()->value("top",5);
	int bottomm = config()->value("bottom",5);

	if (!in.hasChannel(Channel::Depth)) {
		return true;
	}

	ftl::cuda::border_mask(
		out.createTexture<uint8_t>(Channel::Mask, ftl::rgbd::Format<uint8_t>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
		leftm, rightm, topm, bottomm, stream
	);

	return true;
}



CullDiscontinuity::CullDiscontinuity(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

CullDiscontinuity::~CullDiscontinuity() {

}

bool CullDiscontinuity::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Depth) || !in.hasChannel(Channel::Mask)) return false;

	uint8_t maskID = config()->value("mask_id", (unsigned int)(ftl::cuda::Mask::kMask_Discontinuity | ftl::cuda::Mask::kMask_Bad));
	unsigned int radius = config()->value("radius", 2);
	bool inverted = config()->value("invert", false);
	
	out.clearPackets(Channel::Depth);  // Force reset
	ftl::cuda::cull_mask(
		in.createTexture<uint8_t>(Channel::Mask),
		out.createTexture<float>(Channel::Depth),
		maskID,
		inverted,
		radius,
		stream
	);

	return true;
}