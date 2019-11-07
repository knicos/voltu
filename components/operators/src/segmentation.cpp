#include <ftl/operators/segmentation.hpp>
#include "segmentation_cuda.hpp"

using ftl::operators::CrossSupport;
using ftl::operators::VisCrossSupport;
using ftl::codecs::Channel;

CrossSupport::CrossSupport(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

CrossSupport::~CrossSupport() {

}

bool CrossSupport::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	ftl::cuda::support_region(
        in.createTexture<uchar4>(Channel::Colour),
		out.createTexture<uchar4>(Channel::Colour2, ftl::rgbd::Format<uchar4>(in.get<cv::cuda::GpuMat>(Channel::Colour).size())),
		config()->value("tau", 5),
        config()->value("v_max", 5),
        config()->value("h_max", 5), 0
	);

	return true;
}




VisCrossSupport::VisCrossSupport(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

VisCrossSupport::~VisCrossSupport() {

}

bool VisCrossSupport::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	ftl::cuda::vis_support_region(
        in.createTexture<uchar4>(Channel::Colour),
		in.createTexture<uchar4>(Channel::Colour2),
		0
	);

	return true;
}