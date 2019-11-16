#ifndef _FTL_OPERATORS_CLIPPING_HPP_
#define _FTL_OPERATORS_CLIPPING_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace operators {

/**
 * Calculate rough normals from local depth gradients.
 */
class ClipScene : public ftl::operators::Operator {
	public:
    explicit ClipScene(ftl::Configurable*);
    ~ClipScene();

	inline Operator::Type type() const override { return Operator::Type::ManyToMany; }

    bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) override;

};

}
}

#endif  // _FTL_OPERATORS_CLIPPING_HPP_
