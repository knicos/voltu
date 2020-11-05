#include <ftl/operators/mask.hpp>
#include <ftl/operators/cuda/mask.hpp>

using ftl::operators::DiscontinuityMask;
using ftl::operators::BorderMask;
using ftl::operators::CullDiscontinuity;
using ftl::operators::DisplayMask;
using ftl::codecs::Channel;
using ftl::rgbd::Format;

DiscontinuityMask::DiscontinuityMask(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

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

	if (!in.hasChannel(Channel::Depth) || !in.hasChannel(Channel::Support1)) {
		out.message(ftl::data::Message::Warning_MISSING_CHANNEL, "Missing Depth or Support Channel in Mask Operator");
		return false;
	}

	if (!out.hasChannel(Channel::Mask)) {
		cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
		auto &m = out.create<cv::cuda::GpuMat>(Channel::Mask);
		m.create(in.get<cv::cuda::GpuMat>(Channel::Depth).size(), CV_8UC1);
		m.setTo(cv::Scalar(0), cvstream);
	}

	/*ftl::cuda::discontinuity(
		out.createTexture<uint8_t>(Channel::Mask, ftl::rgbd::Format<uint8_t>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
		in.createTexture<uchar4>(Channel::Support1),
		in.createTexture<float>(Channel::Depth),
		in.get<cv::cuda::GpuMat>(Channel::Depth).size(),
		in.getLeftCamera().minDepth, in.getLeftCamera().maxDepth,
		radius, depthCoef, stream
	);*/

	ftl::cuda::discontinuity(
		out.createTexture<uint8_t>(Channel::Mask),
		in.createTexture<uchar4>(Channel::Support1),
		in.createTexture<float>(Channel::Depth),
		in.get<cv::cuda::GpuMat>(Channel::Depth).size(),
		in.getLeftCamera().minDepth, in.getLeftCamera().maxDepth,
		depthCoef, disconThresh, noiseThresh, areaMax, stream
	);

	return true;
}



BorderMask::BorderMask(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

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



CullDiscontinuity::CullDiscontinuity(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

CullDiscontinuity::~CullDiscontinuity() {

}

bool CullDiscontinuity::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Depth) || !in.hasChannel(Channel::Mask)) return false;

	uint8_t maskID = config()->value("mask_id", (unsigned int)(ftl::cuda::Mask::kMask_Discontinuity | ftl::cuda::Mask::kMask_Bad));
	unsigned int radius = config()->value("radius", 2);
	bool inverted = config()->value("invert", false);
	
	out.set<ftl::rgbd::VideoFrame>(Channel::Depth);  // Force reset
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



DisplayMask::DisplayMask(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

DisplayMask::~DisplayMask() {

}

bool DisplayMask::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {

	if (!in.hasChannel(Channel::Mask)) {
		return true;
	}

	uint8_t mask = config()->value("mask", 0);
	bool invert = config()->value("invert", false);

	auto &masktex = in.getTexture<uint8_t>(Channel::Mask);

	if (!in.hasChannel(Channel::Overlay)) {
		auto &t = in.createTexture<uchar4>(Channel::Overlay, ftl::rgbd::Format<uchar4>(masktex.width(), masktex.height()));
		cudaMemset2DAsync(t.devicePtr(), t.pitch(), 0, t.width()*4, t.height(), stream);
	}

	ftl::cuda::show_mask(in.getTexture<uchar4>(Channel::Overlay), masktex, mask, make_uchar4(255,0,255,255), stream);

	return true;
}
