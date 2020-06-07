#include "screencapture.hpp"

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/source.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

using ftl::rgbd::detail::ScreenCapture;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;

#ifdef HAVE_X11
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <X11/extensions/XShm.h>
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
        : ftl::rgbd::detail::Source(host) {
	capabilities_ = kCapVideo;

	ready_ = false;

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

	state_.getLeft() = params_;
	state_.set("name", std::string("[ScreenCapture] ") + host_->value("name", host_->getID()));

	float offsetz = host_->value("offset_z",0.0f);
	state_.setPose(matrix(cv::Vec3d(0.0, 3.14159, 0.0), cv::Vec3d(0.0,0.0,params_.maxDepth+offsetz)));

	host_->on("size", [this](const ftl::config::Event &e) {
		float offsetz = host_->value("offset_z",0.0f);
		params_.maxDepth = host_->value("size", 1.0f);
		state_.getLeft() = params_;
		state_.setPose(matrix(cv::Vec3d(0.0, 3.14159, 0.0), cv::Vec3d(0.0,0.0,params_.maxDepth+offsetz)));
	});

	host_->on("offset_x", [this](const ftl::config::Event &e) {
		offset_x_ = host_->value("offset_x", 0);
	});

	host_->on("offset_y", [this](const ftl::config::Event &e) {
		offset_y_ = host_->value("offset_y", 0);
	});
}

ScreenCapture::~ScreenCapture() {
	#ifdef HAVE_X11
	delete impl_state_;
	#endif
}

bool ScreenCapture::retrieve(ftl::rgbd::Frame &frame) {
	if (!ready_) return false;
	cv::Mat img;

	#ifdef HAVE_X11
	XShmGetImage(impl_state_->display, impl_state_->root, impl_state_->ximg, getOffsetX(), getOffsetY(), 0x00ffffff);
    img = cv::Mat(params_.height, params_.width, CV_8UC4, impl_state_->ximg->data);
	#endif

	frame.reset();
	frame.setOrigin(&state_);

	if (!img.empty()) {
		frame.create<cv::Mat>(Channel::Colour) = img;
	}

	return true;
}

bool ScreenCapture::isReady() {
    return ready_;
}

