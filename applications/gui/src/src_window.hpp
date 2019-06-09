#ifndef _FTL_GUI_SRCWINDOW_HPP_
#define _FTL_GUI_SRCWINDOW_HPP_

#include <nanogui/window.h>
#include <ftl/master.hpp>
#include <ftl/uuid.hpp>
#include <ftl/rgbd/source.hpp>
#include <vector>
#include <string>

class VirtualCameraView;

namespace ftl {
namespace gui {

/**
 * Manage connected nodes and add new connections.
 */
class SourceWindow : public nanogui::Window {
	public:
	SourceWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl);
	~SourceWindow();

	private:
	ftl::ctrl::Master *ctrl_;
	ftl::rgbd::Source *src_;
	VirtualCameraView *image_;
	std::vector<std::string> available_;

};

}
}

#endif  // _FTL_GUI_SRCWINDOW_HPP_
