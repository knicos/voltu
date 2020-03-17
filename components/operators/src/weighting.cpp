#include <ftl/operators/weighting.hpp>
#include "weighting_cuda.hpp"

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::operators::PixelWeights;
using ftl::operators::CullWeight;
using ftl::operators::DegradeWeight;
using ftl::codecs::Channel;

PixelWeights::PixelWeights(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

PixelWeights::~PixelWeights() {

}

bool PixelWeights::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	//if (in.hasChannel(Channel::Mask)) return true;
	
	ftl::cuda::PixelWeightingParameters params;
	//int radius = config()->value("radius", 2);
	float disconPixels = config()->value("discon_pixels", 15.0f);
	params.depthCoef = (1.0f / in.getLeft().fx) * disconPixels;
	params.disconThresh = config()->value("discon_thresh", 0.8f);
	params.noiseThresh = config()->value("noise_thresh", 0.8f);
	params.areaMax = config()->value("area_max", 126.0f);  // Cross support radius squared + 1
	params.depth = config()->value("use_depth", true);
	params.colour = config()->value("use_colour", false);
	params.noise = config()->value("use_noise", true);
	params.normals = config()->value("use_normals", true);
	bool output_normals = config()->value("output_normals", params.normals);

	if ((!in.hasChannel(Channel::Depth) && !in.hasChannel(Channel::GroundTruth)) || !in.hasChannel(Channel::Support1)) return false;

	Channel dchan = (in.hasChannel(Channel::Depth)) ? Channel::Depth : Channel::GroundTruth;

	if (!out.hasChannel(Channel::Mask)) {
		auto &m = out.create<cv::cuda::GpuMat>(Channel::Mask);
		m.create(in.get<cv::cuda::GpuMat>(dchan).size(), CV_8UC1);
		m.setTo(cv::Scalar(0));
	}

	if (output_normals) {
		ftl::cuda::pixel_weighting(
			out.createTexture<short>(Channel::Weights, ftl::rgbd::Format<short>(in.get<cv::cuda::GpuMat>(dchan).size())),
			out.createTexture<uint8_t>(Channel::Mask),
			out.createTexture<half4>(Channel::Normals, ftl::rgbd::Format<half4>(in.get<cv::cuda::GpuMat>(dchan).size())),
			in.createTexture<uchar4>(Channel::Support1),
			in.createTexture<float>(dchan),
			in.getLeftCamera(),
			in.get<cv::cuda::GpuMat>(dchan).size(),
			params, stream
		);
	} else {
		ftl::cuda::pixel_weighting(
			out.createTexture<short>(Channel::Weights, ftl::rgbd::Format<short>(in.get<cv::cuda::GpuMat>(dchan).size())),
			out.createTexture<uint8_t>(Channel::Mask, ftl::rgbd::Format<uint8_t>(in.get<cv::cuda::GpuMat>(dchan).size())),
			in.createTexture<uchar4>(Channel::Support1),
			in.createTexture<float>(dchan),
			in.getLeftCamera(),
			in.get<cv::cuda::GpuMat>(dchan).size(),
			params, stream
		);
	}

	return true;
}

CullWeight::CullWeight(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

CullWeight::~CullWeight() {

}

bool CullWeight::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Depth) || !in.hasChannel(Channel::Weights)) return false;

	float weight = config()->value("weight", 0.1f);
	
	out.clearPackets(Channel::Depth);  // Force reset
	ftl::cuda::cull_weight(
		in.createTexture<short>(Channel::Weights),
		out.createTexture<float>(Channel::Depth),
		weight,
		stream
	);

	return true;
}



DegradeWeight::DegradeWeight(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

DegradeWeight::~DegradeWeight() {

}

bool DegradeWeight::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Mask) || !in.hasChannel(Channel::Weights)) return false;

	uint8_t maskid = config()->value("mask_id", 2);
	int radius = config()->value("radius", 2);
	bool invert = config()->value("invert", false);
	
	ftl::cuda::degrade_mask(
		in.createTexture<uint8_t>(Channel::Mask),
		out.createTexture<short>(Channel::Weights),
		maskid,
		radius,
		invert,
		stream
	);

	return true;
}
