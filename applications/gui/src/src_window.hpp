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
	enum class Mode { rgb, depth, stddev };
	SourceWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl);
	~SourceWindow();

	ftl::rgbd::Source *getSource() const { return src_; }
	bool getDepth() const { return mode_ == Mode::depth; }
	Mode getMode() { return mode_; }

	private:
	ftl::ctrl::Master *ctrl_;
	ftl::rgbd::Source *src_;
	Mode mode_;
	VirtualCameraView *image_;
	std::vector<std::string> available_;

};

}
}

#endif  // _FTL_GUI_SRCWINDOW_HPP_
