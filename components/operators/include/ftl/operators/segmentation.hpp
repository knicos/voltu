#ifndef _FTL_OPERATORS_SEGMENTATION_HPP_
#define _FTL_OPERATORS_SEGMENTATION_HPP_

#include <ftl/operators/operator.hpp>

namespace ftl {
namespace operators {

/**
 * Generate the cross support regions channel.
 */
class CrossSupport : public ftl::operators::Operator {
	public:
    explicit CrossSupport(ftl::Configurable*);
    ~CrossSupport();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

};

/**
 * Visualise the cross support regions channel.
 */
class VisCrossSupport : public ftl::operators::Operator {
	public:
    explicit VisCrossSupport(ftl::Configurable*);
    ~VisCrossSupport();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

};

}
}

#endif  // _FTL_OPERATORS_SEGMENTATION_HPP_
