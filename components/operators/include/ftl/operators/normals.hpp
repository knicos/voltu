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
    explicit Normals(ftl::Configurable*);
    ~Normals();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

};

/**
 * Calculate rough normals from local depth gradients and perform a weighted
 * smoothing over the neighbourhood.
 */
class SmoothNormals : public ftl::operators::Operator {
	public:
    explicit SmoothNormals(ftl::Configurable*);
    ~SmoothNormals();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

	private:
	ftl::cuda::TextureObject<float4> temp_;

};

}
}

#endif  // _FTL_OPERATORS_NORMALS_HPP_
