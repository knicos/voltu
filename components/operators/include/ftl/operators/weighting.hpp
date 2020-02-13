#ifndef _FTL_OPERATORS_WEIGHTING_HPP_
#define _FTL_OPERATORS_WEIGHTING_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace operators {

/**
 * Generate weight values for each pixel in the depth map. At the same time
 * output a mask for the pixel and optionally output normals since they are
 * implicitely calculated anyway.
 * 
 * Weights are based upon:
 * 1) Depth resolution, or distance from the camera
 * 2) Normal direction relative to camera direction
 * 3) Noise or discontinuity assessment.
 * 
 * These weights are then used, for example, by the rendering process when
 * generating depth or colour values from multiple sources.
 */
class PixelWeights : public ftl::operators::Operator {
	public:
	explicit PixelWeights(ftl::Configurable*);
	~PixelWeights();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

class CullWeight : public ftl::operators::Operator {
	public:
	explicit CullWeight(ftl::Configurable*);
	~CullWeight();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

class DegradeWeight : public ftl::operators::Operator {
	public:
	explicit DegradeWeight(ftl::Configurable*);
	~DegradeWeight();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

}
}

#endif  // _FTL_OPERATORS_WEIGHTING_HPP_
