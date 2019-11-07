#pragma once

#include <ftl/operators/operator.hpp>
#include <opencv2/cudaoptflow.hpp>

namespace ftl {
namespace operators {

class NVOpticalFlow : public ftl::operators::Operator {
	public:
	explicit NVOpticalFlow(ftl::Configurable*);
	~NVOpticalFlow();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

	protected:
	void init();

	private:
	cv::Size size_;
	
	// TODO: Left to Flow always assumed, could also calculate something else?
	const ftl::codecs::Channel channel_in_ = ftl::codecs::Channel::Left;
	const ftl::codecs::Channel channel_out_ = ftl::codecs::Channel::Flow;

	cv::Ptr<cv::cuda::NvidiaOpticalFlow_1_0> nvof_;
	cv::cuda::GpuMat left_gray_;
	cv::cuda::GpuMat left_gray_prev_;
};

}
}
