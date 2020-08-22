#include "screencapture.hpp"

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/source.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <ftl/rgbd/capabilities.hpp>
#include <ftl/codecs/touch.hpp>

#include <opencv2/imgproc.hpp>

#include <nlohmann/json.hpp>

using ftl::rgbd::detail::ScreenCapture;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;
using ftl::rgbd::Capability;
using ftl::codecs::Touch;
using ftl::codecs::TouchType;

#ifdef HAVE_X11
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <X11/extensions/XShm.h>
#include <X11/extensions/XTest.h>
#include <sys/ipc.h>
#include <sys/shm.h>

namespace ftl {
namespace rgbd {
namespace detail {
struct X11State {
	Display* display;
    Window root;
    XWindowAttributes window_attributes;
    Screen* screen = window_attributes.screen;
    XShmSegmentInfo shminfo;
    XImage* ximg;
};
}
}
}
#endif

static cv::Mat rmat(const cv::Vec3d &rvec) {
	cv::Mat R(cv::Size(3, 3), CV_64FC1);
	cv::Rodrigues(rvec, R);
	return R;
}

static Eigen::Matrix4d matrix(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
	cv::Mat M = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
	rmat(rvec).copyTo(M(cv::Rect(0, 0, 3, 3)));
	M.at<double>(0, 3) = tvec[0];
	M.at<double>(1, 3) = tvec[1];
	M.at<double>(2, 3) = tvec[2];
	Eigen::Matrix4d r;
	cv::cv2eigen(M,r);
	return r;
}


ScreenCapture::ScreenCapture(ftl::rgbd::Source *host)
        : ftl::rgbd::BaseSourceImpl(host) {
	capabilities_ = kCapVideo;

	ready_ = false;
	primary_touch_.id = -1;

    #ifdef HAVE_X11

	impl_state_ = new X11State;
	auto &s = *impl_state_;

	s.display = XOpenDisplay(NULL);
	if (!s.display) {
		LOG(ERROR) << "Could not open X11 display";
		return;
	}

    s.root = DefaultRootWindow(s.display);  // TODO: Could choose windows?

	int winx = host_->value("x", -1);
	int winy = host_->value("y", -1);

	LOG(INFO) << "WINDOW: " << winx << "," << winy;

	if (winx >= 0 && winy >= 0) {
		Window root;
		Window parent;
		Window *children = 0;
		unsigned int nchildren;
		Status ok = XQueryTree(s.display, s.root, &root, &parent, &children, &nchildren);

		if (ok && nchildren > 0) {
			//s.root = children[nchildren-window-1];

			for (int i=nchildren-1; i>=0; --i) {
				if (XGetWindowAttributes(s.display, children[i], &s.window_attributes)) {
					if ((s.window_attributes.height == 720 || s.window_attributes.height == 1080) && s.window_attributes.map_state == IsViewable) {
						LOG(INFO) << "Possible window: " << s.window_attributes.x << "," << s.window_attributes.y << " " << s.window_attributes.width << "x" << s.window_attributes.height;
						if (s.window_attributes.x <= winx && s.window_attributes.y <= winy) {
							LOG(INFO) << "Found window at " << winx << "," << winy << " : " << i;
							s.root = children[i];
							break;
						}
					}

				}
			}
		}
		XFree(children);
	}

    if (!XGetWindowAttributes(s.display, s.root, &s.window_attributes)) {
		LOG(ERROR) << "Could not get X11 window attributes";
		return;
	}

    s.screen = s.window_attributes.screen;
	full_width_ = s.window_attributes.width;
	full_height_ = s.window_attributes.height;
	offset_x_ = host_->value("offset_x",0);
	offset_y_ = host_->value("offset_y",0);

	// Choose a correct aspect ratio
	int awidth = ftl::codecs::getWidth(ftl::codecs::findDefinition(full_height_));
	params_.width = awidth;
	params_.height = full_height_;

	LOG(INFO) << "Screen capture: " << params_.width << "x" << params_.height;

	s.ximg = XShmCreateImage(s.display, s.window_attributes.visual,
			s.window_attributes.depth, ZPixmap, NULL, &s.shminfo,
			params_.width, params_.height);

	// TODO: Can this happen?
	if (!s.ximg) {
		LOG(ERROR) << "Didn't get shared memory image from X11 screen";
		return;
	}

    s.shminfo.shmid = shmget(IPC_PRIVATE, s.ximg->bytes_per_line * s.ximg->height, IPC_CREAT|0777);
    s.shminfo.shmaddr = s.ximg->data = (char*)shmat(s.shminfo.shmid, 0, 0);
    s.shminfo.readOnly = False;
    if(s.shminfo.shmid < 0) {
        LOG(ERROR) << "Fatal shminfo error!";
		return;
	}

    if (!XShmAttach(impl_state_->display, &impl_state_->shminfo)) {
		LOG(ERROR) << "X11 Shared Memory attach failure";
		return;
	}

	// Page lock the shared memory...
	cudaSafeCall(cudaHostRegister(s.shminfo.shmaddr, s.ximg->bytes_per_line * s.ximg->height, cudaHostRegisterDefault));

	ready_ = true;

	#endif

    params_.cx = -(params_.width / 2.0);
    params_.cy = -(params_.height / 2.0);
    params_.fx = 700.0;
    params_.fy = 700.0;
    params_.maxDepth = host_->value("size", 1.0f);
    params_.minDepth = 0.0f;
	params_.doffs = 0.0;
	params_.baseline = 0.1f;

	do_update_params_ = true;

	//state_.getLeft() = params_;
	//state_.set("name", std::string("[ScreenCapture] ") + host_->value("name", host_->getID()));

	float offsetz = host_->value("offset_z",0.0f);
	//state_.setPose(matrix(cv::Vec3d(0.0, 3.14159, 0.0), cv::Vec3d(0.0,0.0,params_.maxDepth+offsetz)));

	if (host_->getConfig().contains("pose") && host_->getConfig()["pose"].is_array()) {
		LOG(INFO) << "Loading saved screen pose.";
		std::vector<double> d = host_->getConfig()["pose"].get<std::vector<double>>();
		for (int i=0; i<16; ++i) {
			pose_.data()[i] = d[i];
		}
	} else {
		pose_ = matrix(cv::Vec3d(0.0, 3.14159, 0.0), cv::Vec3d(0.0,0.0,params_.maxDepth+offsetz));
	}

	host_->on("size", [this]() {
		float offsetz = host_->value("offset_z",0.0f);
		params_.maxDepth = host_->value("size", 1.0f);
		//state_.getLeft() = params_;
		pose_ = matrix(cv::Vec3d(0.0, 3.14159, 0.0), cv::Vec3d(0.0,0.0,params_.maxDepth+offsetz));
		do_update_params_ = true;
	});

	host_->on("offset_z", [this]() {
		float offsetz = host_->value("offset_z",0.0f);
		params_.maxDepth = host_->value("size", 1.0f);
		//state_.getLeft() = params_;
		pose_ = matrix(cv::Vec3d(0.0, 3.14159, 0.0), cv::Vec3d(0.0,0.0,params_.maxDepth+offsetz));
		do_update_params_ = true;
	});

	host_->on("offset_x", [this]() {
		offset_x_ = host_->value("offset_x", 0);
	});

	host_->on("offset_y", [this]() {
		offset_y_ = host_->value("offset_y", 0);
	});

	host_->on("enable_touch", [this]() {
		do_update_params_ = true;
	});
}

ScreenCapture::~ScreenCapture() {
	#ifdef HAVE_X11
	delete impl_state_;
	#endif
}

/*void ScreenCapture::_mouseClick(int button, int x, int y) {
	#ifdef HAVE_X11

	auto &s = *impl_state_;

	XTestFakeMotionEvent (s.display, 0, x, y, CurrentTime);
	XSync(s.display, 0);

	XTestFakeButtonEvent (s.display, Button1, True,  CurrentTime);
	XTestFakeButtonEvent (s.display, Button1, False, CurrentTime);

	#endif
}*/

void ScreenCapture::_release() {
	pressed_ = false;
	#ifdef HAVE_X11
	auto &s = *impl_state_;
	XTestFakeButtonEvent (s.display, Button1, False, CurrentTime);
	#endif
}

void ScreenCapture::_press() {
	pressed_ = true;
	#ifdef HAVE_X11
	auto &s = *impl_state_;
	XTestFakeButtonEvent (s.display, Button1, True, CurrentTime);
	#endif

	LOG(INFO) << "PRESS";
}

void ScreenCapture::_move(int x, int y) {
	#ifdef HAVE_X11

	auto &s = *impl_state_;
	XTestFakeMotionEvent (s.display, 0, x, y, CurrentTime);
	XSync(s.display, 0);
	#endif
}

void ScreenCapture::_noTouch() {
	if (primary_touch_.id >= 0 && primary_touch_.strength > 0) {
		// RELEASE BUTTON
		_release();
	}
	primary_touch_.id = -1;
}

void ScreenCapture::_singleTouch(const ftl::codecs::Touch &t) {
	// Ignore right clicks currently
	if (t.type != TouchType::MOUSE_LEFT && t.type != TouchType::COLLISION) return;

	if ((primary_touch_.id >= 0 && primary_touch_.id != t.id) || (primary_touch_.id == t.id && primary_touch_.strength > 0 && t.strength == 0)) {
		// RELEASE BUTTON
		_release();
	}

	// Move mouse if no primary or ID is the same.
	if (primary_touch_.id == -1 || t.id == primary_touch_.id) {
		// But only if changed...?
		// MOVE MOUSE
		_move(t.x, t.y);
	}

	// If no primary or same and intensity is > 0, then press
	if ((primary_touch_.id == -1 && t.strength > 0) || (primary_touch_.id == t.id && primary_touch_.strength == 0 && t.strength > 0)) {
		// PRESS EVENT
		_press();
	}

	primary_touch_ = t;
}

void ScreenCapture::_multiTouch(const std::vector<ftl::codecs::Touch> &touches) {

}

bool ScreenCapture::retrieve(ftl::rgbd::Frame &frame) {
	if (!ready_) return false;
	cv::Mat img;

	// TODO: Proper, press, release and motion behaviour
	// Also, render the cursor location

	#ifdef HAVE_X11
	XShmGetImage(impl_state_->display, impl_state_->root, impl_state_->ximg, getOffsetX(), getOffsetY(), 0x00ffffff);
    img = cv::Mat(params_.height, params_.width, CV_8UC4, impl_state_->ximg->data);
	#endif

	if (host_->value("enable_touch", false)) {
		if (frame.changed(Channel::Touch)) {
			const auto &touches = frame.get<std::vector<ftl::codecs::Touch>>(Channel::Touch);
			//LOG(INFO) << "GOT TOUCH DATA " << touches.size();

			/*for (const auto &t : touches) {
				LOG(INFO) << " -- " << t.x << "," << t.y;
			}*/

			if (touches.size() == 0) {
				_noTouch();
			} else if (touches.size() == 1) {
				//_mouseClick(1, touches[0].x, touches[0].y);
				_singleTouch(touches[0]);
			} else if (touches.size() == 2) {
				_multiTouch(touches);
			} else {
				// Too many touches, not supported
			}
		} else {
			_noTouch();
		}

		// If there is a touch, render it.
		if (primary_touch_.id >= 0) {
			if (pressed_) {
				cv::circle(img, cv::Point(primary_touch_.x, primary_touch_.y), 10, cv::Scalar(0,0,255), 5);
			} else {
				cv::circle(img, cv::Point(primary_touch_.x, primary_touch_.y), 10, cv::Scalar(0,0,255), 3);
			}
		}
	}

	if (frame.changed(Channel::Pose)) {
		LOG(INFO) << "Pose has been updated...";
		Eigen::Matrix4d p = frame.getPose();
		std::vector<double> d;
		d.resize(16);
		for (int i=0; i<16; ++i) {
			d[i] = p.data()[i];
		}
		host_->getConfig()["pose"] = d;
	}

	if (do_update_params_) {
		frame.setPose() = pose_;
		frame.setLeft() = params_;

		auto &meta = frame.create<std::map<std::string,std::string>>(Channel::MetaData);
		meta["name"] = host_->value("name", host_->getID());
		meta["id"] = host_->getID();
		meta["uri"] = host_->value("uri", std::string(""));
		meta["device"] = std::string("X11 Screen Capture");

		//if (!frame.has(Channel::Capabilities)) {
			auto &cap = frame.create<std::unordered_set<Capability>>(Channel::Capabilities);
			cap.clear();
			cap.emplace(Capability::VIDEO);
			cap.emplace(Capability::LIVE);
			if (host_->value("enable_touch", false)) cap.emplace(Capability::TOUCH);
		//}

		do_update_params_ = false;
	}

	if (!img.empty()) {
		frame.create<cv::cuda::GpuMat>(Channel::Colour).upload(img);
	}

	return true;
}

bool ScreenCapture::isReady() {
    return ready_;
}

