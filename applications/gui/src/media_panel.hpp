#ifndef _FTL_GUI_MEDIAPANEL_HPP_
#define _FTL_GUI_MEDIAPANEL_HPP_

#include "camera.hpp"

#include <nanogui/window.h>

#include "src_window.hpp"

#include <array>

namespace ftl {

namespace rgbd {
class SnapshotStreamWriter;
}

namespace gui {

class Screen;

class MediaPanel : public nanogui::Window {
	public:
	explicit MediaPanel(ftl::gui::Screen *, ftl::gui::SourceWindow *);
	~MediaPanel();

	void cameraChanged();

	//void startRecording2D(ftl::gui::Camera *camera, const std::string &filename);

	//void snapshot3D(ftl::gui::Camera *camera, const std::string &filename);

	//void startRecording3D(ftl::gui::Camera *camera, const std::string &filename);

	void recordWindowClosed();

	void performLayout(NVGcontext *ctx) override;

	private:
	ftl::gui::Screen *screen_;
	ftl::gui::SourceWindow *sourceWindow_;

	bool paused_;
	bool disable_switch_channels_;

	ftl::rgbd::SnapshotStreamWriter *writer_;
	nanogui::PopupButton *button_channels_;
	//nanogui::Button *right_button_;
	//nanogui::Button *depth_button_;
	nanogui::Popup *more_button_;
	nanogui::PopupButton *recordbutton_;
	std::array<nanogui::Button*,32> channel_buttons_={};

	enum class RecordMode {
		None,
		Snapshot2D,
		Snapshot3D,
		Video2D,
		Video3D,
		Live2D,
		Live3D
	};
	RecordMode record_mode_;

	void _startRecording(RecordMode mode);
	void _stopRecording();

	/**
	 * These members indicate which type of recording is active, if any.
	 * They also include a pointer to an object which is used
	 * to end the recording. Only one of these members should have a value
	 * at any given time.
	 */
	//std::optional<ftl::gui::Camera*> virtualCameraRecording_;
	//std::optional<ftl::Configurable*> sceneRecording_;
};

}
}

#endif  // _FTL_GUI_MEDIAPANEL_HPP_
