#ifndef _FTL_OPERATORS_MVMLS_HPP_
#define _FTL_OPERATORS_MVMLS_HPP_

#include <ftl/operators/operator.hpp>

namespace ftl {
namespace operators {

class MultiViewMLS : public ftl::operators::Operator {
	public:
	explicit MultiViewMLS(ftl::Configurable*);
	~MultiViewMLS();

	inline Operator::Type type() const override { return Operator::Type::ManyToMany; }

	bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) override;

	private:
	std::vector<ftl::cuda::TextureObject<float4>> centroid_horiz_;
	std::vector<ftl::cuda::TextureObject<float4>> centroid_vert_;
	std::vector<ftl::cuda::TextureObject<float4>> normals_horiz_;
    std::vector<ftl::cuda::TextureObject<float>> contributions_;
};

}
}

#endif
