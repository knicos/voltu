#ifndef _FTL_GUI_SRCWINDOW_HPP_
#define _FTL_GUI_SRCWINDOW_HPP_

#include <nanogui/window.h>
#include <nanogui/imageview.h>
#include <ftl/master.hpp>
#include <ftl/uuid.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/threads.hpp>
#include <vector>
#include <map>
#include <string>
#include "gltexture.hpp"

class VirtualCameraView;

namespace ftl {
namespace gui {

class Screen;
class Camera;

class SourceWindow : public nanogui::Window {
	public:
	SourceWindow(ftl::gui::Screen *screen);
	~SourceWindow();

	const std::vector<ftl::gui::Camera*> &getCameras();

	virtual void draw(NVGcontext *ctx);

	private:
	ftl::gui::Screen *screen_;
	std::map<std::string, ftl::gui::Camera*> cameras_; 
	std::vector<std::string> available_;
	std::vector<GLTexture> thumbs_;
	bool refresh_thumbs_;
	nanogui::Widget *ipanel_;
	std::mutex mutex_;

	void _updateCameras();

};

}
}

#endif  // _FTL_GUI_SRCWINDOW_HPP_
