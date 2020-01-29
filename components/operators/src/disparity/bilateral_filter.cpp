#include <ftl/operators/disparity.hpp>

#include "opencv/joint_bilateral.hpp"
#include "cuda.hpp"

using cv::cuda::GpuMat;
using cv::Size;

using ftl::codecs::Channel;
using ftl::operators::DisparityBilateralFilter;

DisparityBilateralFilter::DisparityBilateralFilter(ftl::Configurable* cfg) :
		ftl::operators::Operator(cfg) {
	
	scale_ = 16.0;
	n_disp_ = cfg->value("n_disp", 256);
	radius_ = cfg->value("radius", 7);
	iter_ = cfg->value("iter", 13);
	filter_ = nullptr;
}

bool DisparityBilateralFilter::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out,
									 cudaStream_t stream) {

	if (!in.hasChannel(Channel::Colour)) {
		throw FTL_Error("Joint Bilateral Filter is missing Colour");
		return false;
	} else if (!in.hasChannel(Channel::Disparity)) {
		// Have depth, so calculate disparity...
		if (in.hasChannel(Channel::Depth)) {
			// No disparity, so create it.
			const auto params = in.getLeftCamera();
			const GpuMat &depth = in.get<GpuMat>(Channel::Depth);

			GpuMat &disp = out.create<GpuMat>(Channel::Disparity);
			disp.create(depth.size(), CV_32FC1);

			//LOG(ERROR) << "Calculated disparity from depth";

			ftl::cuda::depth_to_disparity(disp, depth, params, stream);
		} else {
			throw FTL_Error("Joint Bilateral Filter is missing Disparity and Depth");
			return false;
		}
	}

	if (!filter_) filter_ = ftl::cuda::createDisparityBilateralFilter(n_disp_ * scale_, radius_, iter_);

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	const GpuMat &rgb = in.get<GpuMat>(Channel::Colour);
	GpuMat &disp_in = in.get<GpuMat>(Channel::Disparity);
	GpuMat &disp_out = out.create<GpuMat>(Channel::Disparity);
	disp_out.create(disp_in.size(), disp_in.type());

	disp_in.convertTo(disp_int_, CV_16SC1, scale_, cvstream);
	filter_->apply(disp_int_, rgb, disp_int_result_, cvstream);
	disp_int_result_.convertTo(disp_out, disp_in.type(), 1.0/scale_, cvstream);
	return true;
}