#ifndef _FTL_OPERATORS_ANTIALIASING_HPP_
#define _FTL_OPERATORS_ANTIALIASING_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace operators {

/**
 * Fast Approximate Anti-Aliasing by NVIDIA (2010)
 */
class FXAA : public ftl::operators::Operator {
	public:
    explicit FXAA(ftl::Configurable*);
    ~FXAA();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

}
}

#endif  // _FTL_OPERATORS_ANTIALIASING_HPP_
