#include <ftl/operators/segmentation.hpp>
#include "segmentation_cuda.hpp"

using ftl::operators::CrossSupport;
using ftl::operators::VisCrossSupport;
using ftl::codecs::Channel;

CrossSupport::CrossSupport(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

CrossSupport::~CrossSupport() {

}

bool CrossSupport::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	bool use_mask = config()->value("discon_support", false);


	if (use_mask && !in.hasChannel(Channel::Support2)) {
		if (!in.hasChannel(Channel::Mask)) return false;
		ftl::cuda::support_region(
			in.createTexture<int>(Channel::Mask),
			out.createTexture<uchar4>(Channel::Support2, ftl::rgbd::Format<uchar4>(in.get<cv::cuda::GpuMat>(Channel::Colour).size())),
			config()->value("v_max", 5),
			config()->value("h_max", 5),
			config()->value("symmetric", false), stream
		);
	} else if (!in.hasChannel(Channel::Support1)) {
		ftl::cuda::support_region(
			in.createTexture<uchar4>(Channel::Colour),
			out.createTexture<uchar4>(Channel::Support1, ftl::rgbd::Format<uchar4>(in.get<cv::cuda::GpuMat>(Channel::Colour).size())),
			config()->value("tau", 5.0f),
			config()->value("v_max", 5),
			config()->value("h_max", 5),
			config()->value("symmetric", true), stream
		);
	}

	return true;
}




VisCrossSupport::VisCrossSupport(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

VisCrossSupport::~VisCrossSupport() {

}

bool VisCrossSupport::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	bool show_depth = false;
	if (in.hasChannel(Channel::Support2) && config()->value("show_depth_support", false)) {
		show_depth = true;
	}

	bool show_bad = config()->value("show_bad", false) && in.hasChannel(Channel::Support2);

	if (show_bad) {
		ftl::cuda::vis_bad_region(
			in.createTexture<uchar4>(Channel::Colour),
			in.createTexture<float>(Channel::Depth),
			in.createTexture<uchar4>(Channel::Support1),
			in.createTexture<uchar4>(Channel::Support2),
			stream
		);
	} else {
		ftl::cuda::vis_support_region(
			in.createTexture<uchar4>(Channel::Colour),
			in.createTexture<uchar4>(Channel::Support1),
			make_uchar4(0,0,255,0),
			make_uchar4(255,0,0,0),
			config()->value("offset_x", 0),
			config()->value("offset_y", 0),
			config()->value("spacing_x", 50),
			config()->value("spacing_y", 50),
			stream
		);

		if (show_depth) {
			ftl::cuda::vis_support_region(
				in.createTexture<uchar4>(Channel::Colour),
				in.createTexture<uchar4>(Channel::Support2),
				make_uchar4(0,0,255,0),
				make_uchar4(0,255,0,0),
				config()->value("offset_x", 0),
				config()->value("offset_y", 0),
				config()->value("spacing_x", 50),
				config()->value("spacing_y", 50),
				stream
			);
		}
	}

	return true;
}