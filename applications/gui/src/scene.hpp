#ifndef _FTL_GUI_SCENE_HPP_
#define _FTL_GUI_SCENE_HPP_

#include <ftl/streams/receiver.hpp>

namespace ftl {
namespace gui {

class Camera;

class Scene {
	public:
	explicit Scene(ftl::stream::Receiver *);
	~Scene();

	inline const std::vector<ftl::gui::Camera*> cameras() const { return cameras_; };

	private:
	std::vector<ftl::gui::Camera*> cameras_;
};

}
}

#endif  // _FTL_GUI_SCENE_HPP_
