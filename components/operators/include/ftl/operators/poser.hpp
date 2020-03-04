#ifndef _FTL_OPERATORS_POSER_HPP_
#define _FTL_OPERATORS_POSER_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/cuda_common.hpp>

#include <unordered_map>

namespace ftl {
namespace operators {

/**
 * Cache and apply poses from and to framesets.
 */
class Poser : public ftl::operators::Operator {
	public:
	explicit Poser(ftl::Configurable*);
	~Poser();

	inline Operator::Type type() const override { return Operator::Type::ManyToMany; }

	bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) override;

    private:
	struct PoseState {
		Eigen::Matrix4d pose;
		bool locked;
	};

    static std::unordered_map<std::string,PoseState> pose_db__;

};

}
}

#endif
