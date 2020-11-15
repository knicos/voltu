#ifndef _FTL_OPERATORS_FUSION_HPP_
#define _FTL_OPERATORS_FUSION_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/operators/cuda/mls/multi_intensity.hpp>
#include <vector>

namespace ftl {
namespace operators {

class Fusion : public ftl::operators::Operator {
	public:
	Fusion(ftl::operators::Graph *g, ftl::Configurable*);
	~Fusion();

	inline Operator::Type type() const override { return Operator::Type::ManyToMany; }

	bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) override;

	static void configuration(ftl::Configurable*);

	private:
	ftl::cuda::MLSMultiIntensity mls_;
	std::vector<cv::cuda::GpuMat> weights_;
	cv::cuda::GpuMat temp_;
	cv::cuda::GpuMat temp2_;
};

}
}

#endif
