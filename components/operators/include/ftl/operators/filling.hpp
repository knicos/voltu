#ifndef _FTL_OPERATORS_FILLING_HPP_
#define _FTL_OPERATORS_FILLING_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace operators {

/**
 * Use colour distance field to scanline correct depth edges and fill holes.
 */
class ScanFieldFill : public ftl::operators::Operator {
	public:
    explicit ScanFieldFill(ftl::Configurable*);
    ~ScanFieldFill();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

class CrossSupportFill : public ftl::operators::Operator {
	public:
    explicit CrossSupportFill(ftl::Configurable*);
    ~CrossSupportFill();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

}
}

#endif  // _FTL_OPERATORS_FILLING_HPP_
