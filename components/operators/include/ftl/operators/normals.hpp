#ifndef _FTL_OPERATORS_NORMALS_HPP_
#define _FTL_OPERATORS_NORMALS_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace operators {

/**
 * Calculate rough normals from local depth gradients.
 */
class Normals : public ftl::operators::Operator {
	public:
    Normals(ftl::operators::Graph *g, ftl::Configurable*);
    ~Normals();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

/**
 * Calculate the dot product of the normal and a camera ray. Outputs a single
 * float value between 1 and -1 with 1 being facing camera, 0 being
 * perpendicular and -1 facing wrong direction. Useful during rendering where
 * full normals are not required.
 */
class NormalDot : public ftl::operators::Operator {
	public:
    NormalDot(ftl::operators::Graph *g, ftl::Configurable*);
    ~NormalDot();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

};

/**
 * Calculate rough normals from local depth gradients and perform a weighted
 * smoothing over the neighbourhood.
 */
class SmoothNormals : public ftl::operators::Operator {
	public:
    SmoothNormals(ftl::operators::Graph *g, ftl::Configurable*);
    ~SmoothNormals();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	ftl::cuda::TextureObject<half4> temp_;

};

}
}

#endif  // _FTL_OPERATORS_NORMALS_HPP_
