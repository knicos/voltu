#ifndef _FTL_OPERATORS_GTANALYSIS_HPP_
#define _FTL_OPERATORS_GTANALYSIS_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/cuda_common.hpp>
#include <ftl/operators/cuda/gt.hpp>

namespace ftl {
namespace operators {

/**
 * Ground Truth analysis, applys indications and metrics that compare any
 * depth map with a ground truth channel (if both exist).
 */
class GTAnalysis : public ftl::operators::Operator {
	public:
    GTAnalysis(ftl::operators::Graph *g, ftl::Configurable*);
    ~GTAnalysis();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	static void configuration(ftl::Configurable*);

	private:
	ftl::cuda::GTAnalysisData *output_;
};

}
}

#endif  // _FTL_OPERATORS_GTANALYSIS_HPP_
