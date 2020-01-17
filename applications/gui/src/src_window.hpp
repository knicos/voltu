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

#include <ftl/streams/stream.hpp>
#include <ftl/streams/receiver.hpp>
#include <ftl/streams/filestream.hpp>

class VirtualCameraView;

namespace ftl {
namespace gui {

class Screen;
class Scene;
class Camera;
class ThumbView;

/**
 * Main class for managing all data streams and corresponding cameras. It
 * will automatically locate all available streams and generate default cameras
 * for each frame of each stream found. It will also add a single default
 * virtual camera. Additional cameras can be added. This class directly
 * receives all frameset data and then forwards it to the individual cameras
 * for drawing/rendering.
 */
class SourceWindow : public nanogui::Window {
	public:
	explicit SourceWindow(ftl::gui::Screen *screen);
	~SourceWindow();

	std::vector<ftl::gui::Camera*> getCameras();

	virtual void draw(NVGcontext *ctx);

	void recordVideo(const std::string &filename);
	void stopRecordingVideo();

	private:
	ftl::gui::Screen *screen_;

	struct CameraEntry {
		ftl::gui::Camera *camera;
		ftl::gui::ThumbView *thumbview;
		//GLTexture thumb;
	};

	std::map<int, CameraEntry> cameras_;
	ftl::stream::Muxer *stream_;
	ftl::stream::Intercept *interceptor_;
	ftl::stream::File *recorder_;
	ftl::stream::Receiver *receiver_;
	std::vector<std::string> available_;
	std::vector<GLTexture> thumbs_;
	bool refresh_thumbs_;
	nanogui::Widget *ipanel_;
	int cycle_;
	ftl::operators::Graph *pre_pipeline_;
	MUTEX mutex_;

	void _updateCameras(const std::vector<std::string> &netcams);
	void _createDefaultCameras(ftl::rgbd::FrameSet &fs, bool makevirtual);
	ftl::codecs::Channels<0> _aggregateChannels();

};

}
}

#endif  // _FTL_GUI_SRCWINDOW_HPP_
