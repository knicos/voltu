#ifndef _FTL_GUI_POSEWINDOW_HPP_
#define _FTL_GUI_POSEWINDOW_HPP_

#include <nanogui/window.h>
#include <ftl/master.hpp>
#include <ftl/uuid.hpp>

namespace ftl {
namespace gui {

class Screen;

/**
 * Manage connected nodes and add new connections.
 */
class PoseWindow : public nanogui::Window {
	public:
	PoseWindow(ftl::gui::Screen *screen, const std::string &src);
	~PoseWindow();

	private:
	std::vector<std::string> available_;
	std::string src_;

	enum poseparameter_t {
		kPoseTranslation,
		kPoseRotation,
		kPoseRaw
	};

	poseparameter_t pose_param_;
	float pose_precision_;
	Eigen::Matrix4d pose_;
	ftl::gui::Screen *screen_;
	bool poselink_;
};

}
}

#endif  // _FTL_GUI_POSEWINDOW_HPP_
