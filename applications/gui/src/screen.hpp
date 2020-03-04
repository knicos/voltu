#ifndef _FTL_GUI_SCREEN_HPP_
#define _FTL_GUI_SCREEN_HPP_

#include <nanogui/screen.h>
#include <nanogui/glutil.h>
#include <ftl/master.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/configuration.hpp>

#include "ctrl_window.hpp"
#include "src_window.hpp"
#include "gltexture.hpp"

#ifdef HAVE_OPENVR
#include <openvr/openvr.h>
#endif

class StatisticsImageNSamples;

namespace ftl {
namespace gui {

class Camera;
class MediaPanel;

struct Statistics {
	float fps;
	float latency;
	float bitrate;
};

class Screen : public nanogui::Screen {
	public:
	explicit Screen(ftl::Configurable *root, ftl::net::Universe *net, ftl::ctrl::Master *controller);
	~Screen();

	bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers);
	bool scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel);
	bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers);
	bool keyboardEvent(int key, int scancode, int action, int modifiers);

	void setActivePose(const Eigen::Matrix4d &p);

	virtual void draw(NVGcontext *ctx);

	void drawFast();

	ftl::Configurable *root() { return root_; }
	ftl::net::Universe *net() { return net_; }
	ftl::ctrl::Master *control() { return ctrl_; }

	void setActiveCamera(ftl::gui::Camera*);
	ftl::gui::Camera *activeCamera() { return camera_; }

	const ftl::gui::Statistics &getStatistics();

#ifdef HAVE_OPENVR
	// initialize OpenVR
	bool initVR();

	// is VR mode on/off
	bool isVR();

	// toggle VR on/off
	bool switchVR(bool mode);

	bool isHmdPresent();

	vr::IVRSystem* getVR() { return HMD_; }

#else
	bool isVR() { return false; }
#endif

	nanogui::Theme *windowtheme;
	nanogui::Theme *specialtheme;
	nanogui::Theme *mediatheme;
	nanogui::Theme *toolbuttheme;

	private:
	ftl::gui::SourceWindow *swindow_;
	ftl::gui::ControlWindow *cwindow_;
	ftl::gui::MediaPanel *mwindow_;

	//std::vector<SourceViews> sources_;
	ftl::net::Universe *net_;
	nanogui::GLShader mShader;
	GLuint mImageID;
	//Source *src_;
	//GLTexture texture_;
	Eigen::Vector3f eye_;
	Eigen::Vector4f neye_;
	Eigen::Vector3f orientation_;
	Eigen::Vector3f up_;
	//Eigen::Vector3f lookPoint_;
	float lerpSpeed_;
	bool depth_;
	float ftime_;
	float delta_;
	Eigen::Vector2f imageSize;
	ftl::ctrl::Master *ctrl_;
	ftl::Configurable *root_;
	std::string status_;
	ftl::gui::Camera *camera_;
	float zoom_;
	float pos_x_;
	float pos_y_;

	GLuint leftEye_;
	GLuint rightEye_;

	std::vector<cv::Point3d> points_;

	bool show_two_images_ = false;

	ftl::Configurable *shortcuts_;

	#ifdef HAVE_OPENVR
	vr::IVRSystem *HMD_;
	#endif

	ftl::gui::Statistics stats_;
	int last_stats_count_;
};

}
}

#endif  // _FTL_GUI_SCREEN_HPP_
