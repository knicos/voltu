#ifndef _FTL_SCENE_FRAMESCENE_HPP_
#define _FTL_SCENE_FRAMESCENE_HPP_

#include <ftl/scene/scene.hpp>

namespace ftl {
namespace scene {

/**
 * A scene represented internally as a set of image frames that together
 * define a point cloud.
 */
class FrameScene : public ftl::scene::Scene {
	public:
	FrameScene();
	~FrameScene();

	bool update(ftl::rgbd::FrameSet &);

	bool render(ftl::rgbd::Source *, ftl::rgbd::Frame &);
	bool encode(std::vector<uint8_t> &);
	bool decode(const std::vector<uint8_t> &);
};

}
}

#endif  // _FTL_SCENE_FRAMESCENE_HPP_
