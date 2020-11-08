#pragma once

#include <ftl/operators/operator.hpp>
#include <opencv2/cudaoptflow.hpp>

namespace ftl {
namespace operators {

/*
 * Compute Optical flow from Channel::Colour (Left) and save the result in
 * Channel::Flow using NVidia Optical Flow 1.0 (via OpenCV wrapper).
 */
class NVOpticalFlow : public ftl::operators::Operator {
	public:
	NVOpticalFlow(ftl::operators::Graph *g, ftl::Configurable*);
	NVOpticalFlow(ftl::operators::Graph *g, ftl::Configurable*, const std::tuple<ftl::codecs::Channel,ftl::codecs::Channel,ftl::codecs::Channel,ftl::codecs::Channel> &channels);
	~NVOpticalFlow();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	static void configuration(ftl::Configurable*) {}

	protected:
	bool init();

	private:
	cv::Size size_;
	
	// TODO: Colour to Flow always assumed, could also calculate something else?
	ftl::codecs::Channel channel_in_[2];
	ftl::codecs::Channel channel_out_[2];

	cv::Ptr<cv::cuda::NvidiaOpticalFlow_1_0> nvof_;
	cv::cuda::GpuMat left_gray_;
	cv::cuda::GpuMat left_gray_prev_;
	cv::cuda::GpuMat right_gray_;
	cv::cuda::GpuMat right_gray_prev_;
};

}
}
