#ifndef _FTL_GUI_MEDIAPANEL_HPP_
#define _FTL_GUI_MEDIAPANEL_HPP_

#include <nanogui/window.h>

namespace ftl {

namespace rgbd {
class SnapshotStreamWriter;
}

namespace gui {

class Screen;

class MediaPanel : public nanogui::Window {
	public:
	explicit MediaPanel(ftl::gui::Screen *);
	~MediaPanel();

	void cameraChanged();

	private:
	ftl::gui::Screen *screen_;

	bool paused_;
	bool disable_switch_channels_;

	ftl::rgbd::SnapshotStreamWriter *writer_;
	nanogui::PopupButton *button_channels_;
	nanogui::Button *right_button_;
	nanogui::Button *depth_button_;
};

}
}

#endif  // _FTL_GUI_MEDIAPANEL_HPP_
