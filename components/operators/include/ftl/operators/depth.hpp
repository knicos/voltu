#pragma once

#include <ftl/config.h>
#include <ftl/operators/operator.hpp>
#include <opencv2/core.hpp>


namespace ftl {
namespace operators {

class DepthBilateralFilter : public::ftl::operators::Operator {
	public:
	explicit DepthBilateralFilter(ftl::operators::Graph *g, ftl::Configurable*);
	DepthBilateralFilter(ftl::operators::Graph *g, ftl::Configurable*, const std::tuple<ftl::codecs::Channel> &);

	~DepthBilateralFilter() {};

	inline ftl::operators::Operator::Type type() const override { return ftl::operators::Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	static void configuration(ftl::Configurable*) {}

	private:
	cv::cuda::GpuMat disp_int_;
	cv::cuda::GpuMat disp_int_result_;
	double scale_;
	int radius_;
	int iter_;
	float edge_disc_;
	float max_disc_;
	float sigma_range_;
	ftl::codecs::Channel channel_;

    cv::cuda::GpuMat table_color_;
	cv::cuda::GpuMat table_space_;
};

}
}
