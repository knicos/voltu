#ifndef _FTL_GUI_SRCWINDOW_HPP_
#define _FTL_GUI_SRCWINDOW_HPP_

#include <nanogui/window.h>
#include <ftl/master.hpp>
#include <ftl/uuid.hpp>
#include <ftl/net_source.hpp>

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
	ftl::rgbd::NetSource *src_;
	VirtualCameraView *image_;

};

}
}

#endif  // _FTL_GUI_SRCWINDOW_HPP_
