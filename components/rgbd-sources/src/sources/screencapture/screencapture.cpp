#include "screencapture.hpp"

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/source.hpp>

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

ScreenCapture::ScreenCapture(ftl::rgbd::Source *host)
        : ftl::rgbd::detail::Source(host) {
	capabilities_ = kCapVideo;

	const uint WIDTH  = 1280;
	const uint HEIGHT = 720;

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

    if (!XGetWindowAttributes(s.display, s.root, &s.window_attributes)) {
		LOG(ERROR) << "Could not get X11 window attributes";
		return;
	}

    s.screen = s.window_attributes.screen;
	params_.width = s.window_attributes.width;
	params_.height = s.window_attributes.height;

    s.ximg = XShmCreateImage(s.display, DefaultVisualOfScreen(s.screen),
			DefaultDepthOfScreen(s.screen), ZPixmap, NULL, &s.shminfo,
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

	ready_ = true;

	#endif

    params_.cx = -(params_.width / 2.0);
    params_.cy = -(params_.height / 2.0);
    params_.fx = 700.0;
    params_.fy = 700.0;
    params_.maxDepth = host_->value("depth", 1.0f);
    params_.minDepth = 0.0f;
	params_.doffs = 0.0;

	state_.getLeft() = params_;

}

ScreenCapture::~ScreenCapture() {
	#ifdef HAVE_X11
	delete impl_state_;
	#endif
}

void ScreenCapture::swap() {
	cur_ts_ = cap_ts_;
	sframe_.swapTo(frame_);
}

bool ScreenCapture::retrieve() {
	if (!ready_) return false;
	cv::Mat img;

	#ifdef HAVE_X11
	XShmGetImage(impl_state_->display, impl_state_->root, impl_state_->ximg, 0, 0, 0x00ffffff);
    img = cv::Mat(params_.height, params_.width, CV_8UC4, impl_state_->ximg->data);
	#endif

	sframe_.reset();
	sframe_.setOrigin(&state_);

	if (!img.empty()) {
		sframe_.create<cv::cuda::GpuMat>(Channel::Colour).upload(img);
	}

	cap_ts_ = timestamp_;

	return true;
}

bool ScreenCapture::compute(int n, int b) {
	if (!ready_) return false;
	host_->notify(cur_ts_, frame_);
    return true;
}

bool ScreenCapture::isReady() {
    return ready_;
}
