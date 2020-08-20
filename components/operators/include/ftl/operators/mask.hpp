#ifndef _FTL_OPERATORS_MASK_HPP_
#define _FTL_OPERATORS_MASK_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace operators {

/**
 * Generate a masking channel that indicates depth discontinuities within a
 * specified radius from a pixel. This is useful for culling potentially bad
 * depth and colour values when merging and smoothing.
 */
class DiscontinuityMask : public ftl::operators::Operator {
	public:
	DiscontinuityMask(ftl::operators::Graph *g, ftl::Configurable*);
	~DiscontinuityMask();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

/**
 * Generate a depth border mask.
 */
class BorderMask : public ftl::operators::Operator {
	public:
	BorderMask(ftl::operators::Graph *g, ftl::Configurable*);
	~BorderMask();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

/**
 * Visualise a mask value
 */
class DisplayMask : public ftl::operators::Operator {
	public:
	DisplayMask(ftl::operators::Graph *g, ftl::Configurable*);
	~DisplayMask();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

/**
 * Remove depth values marked with the discontinuity mask.
 */
class CullDiscontinuity : public ftl::operators::Operator {
	public:
	CullDiscontinuity(ftl::operators::Graph *g, ftl::Configurable*);
	~CullDiscontinuity();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

}
}

#endif  // _FTL_OPERATORS_MASK_HPP_
