#ifndef _FTL_OPERATORS_POSER_HPP_
#define _FTL_OPERATORS_POSER_HPP_

#include <ftl/operators/operator.hpp>
#include <ftl/cuda_common.hpp>
#include <ftl/codecs/shapes.hpp>

#include <unordered_map>

namespace ftl {
namespace operators {

/**
 * Cache and apply poses from and to framesets.
 */
class Poser : public ftl::operators::Operator {
	public:
	Poser(ftl::operators::Graph *g, ftl::Configurable*);
	~Poser();

	inline Operator::Type type() const override { return Operator::Type::ManyToMany; }

	bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) override;

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	static bool get(const std::string &name, Eigen::Matrix4d &pose);
	static bool get(const std::string &name, ftl::codecs::Shape3D &shape);
	static std::list<ftl::codecs::Shape3D*> getAll(int32_t fsid);

	//static bool set(const ftl::codecs::Shape3D &shape);

	static void add(const ftl::codecs::Shape3D &t, ftl::data::FrameID id);

    private:
	struct PoseState {
		ftl::codecs::Shape3D shape;
		bool locked;
	};

    static std::unordered_map<std::string,PoseState> pose_db__;
	static std::unordered_map<int,std::list<ftl::codecs::Shape3D*>> fs_shapes__;
};

}
}

#endif
