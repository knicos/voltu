#pragma once

#include <ftl/operators/operator.hpp>
#include <opencv2/cudaoptflow.hpp>

#include <libsgm.h>

namespace ftl {
namespace operators {

/*
 * FixstarsSGM https://github.com/fixstars/libSGM
 * 
 * Requires modified version https://gitlab.utu.fi/joseha/libsgm
 * 
 */
class FixstarsSGM : public ftl::operators::Operator {
	public:
	explicit FixstarsSGM(ftl::Configurable* cfg);

	~FixstarsSGM();
	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;
	
	private:
	bool init();
	bool updateParameters();
	
	sgm::StereoSGM *ssgm_;
	cv::Size size_;
	cv::cuda::GpuMat lbw_;
	cv::cuda::GpuMat rbw_;
	cv::cuda::GpuMat disp_int_;

	int P1_;
	int P2_;
	int max_disp_;
	float uniqueness_;
};

class DisparityBilateralFilter : public::ftl::operators::Operator {
	public:
	explicit DisparityBilateralFilter(ftl::Configurable*);

	~DisparityBilateralFilter() {};

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

	private:
	cv::Ptr<cv::cuda::DisparityBilateralFilter> filter_;
	cv::cuda::GpuMat disp_int_;
	cv::cuda::GpuMat disp_int_result_;
	double scale_;
	int radius_;
	int iter_;
	int n_disp_;
};

/*
 * Calculate depth from disparity
 */
class DisparityToDepth : public ftl::operators::Operator {
	public:
	explicit DisparityToDepth(ftl::Configurable* cfg) :
		ftl::operators::Operator(cfg) {}

	~DisparityToDepth() {};
	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;
};

/*
 * Optical flow smoothing for depth
 */
class OpticalFlowTemporalSmoothing : public ftl::operators::Operator {
	public:
	explicit OpticalFlowTemporalSmoothing(ftl::Configurable*);
	~OpticalFlowTemporalSmoothing();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

	private:
	bool init();

	const ftl::codecs::Channel channel_ = ftl::codecs::Channel::Disparity;
	cv::cuda::GpuMat history_;
	cv::Size size_;
	int n_max_;
	float threshold_;
};

}
}
