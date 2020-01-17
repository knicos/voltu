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
	explicit NVOpticalFlow(ftl::Configurable*);
	~NVOpticalFlow();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	protected:
	bool init();

	private:
	cv::Size size_;
	
	// TODO: Colour to Flow always assumed, could also calculate something else?
	const ftl::codecs::Channel channel_in_ = ftl::codecs::Channel::Colour;
	const ftl::codecs::Channel channel_out_ = ftl::codecs::Channel::Flow;

	cv::Ptr<cv::cuda::NvidiaOpticalFlow_1_0> nvof_;
	cv::cuda::GpuMat left_gray_;
	cv::cuda::GpuMat left_gray_prev_;
};

}
}
