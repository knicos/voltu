#include <loguru.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/threads.hpp>

//#include "sources/net/net.hpp"
#include "sources/stereovideo/stereovideo.hpp"
#include "sources/image/image.hpp"
#include "sources/middlebury/middlebury_source.hpp"
#include "sources/screencapture/screencapture.hpp"

#ifdef HAVE_LIBARCHIVE
#include <ftl/rgbd/snapshot.hpp>
#include "sources/snapshot/snapshot_source.hpp"
#endif

//#include "sources/ftlfile/file_source.hpp"

#ifdef HAVE_REALSENSE
#include "sources/realsense/realsense_source.hpp"
using ftl::rgbd::detail::RealsenseSource;
#endif

#ifdef HAVE_PYLON
#include "sources/pylon/pylon.hpp"
using ftl::rgbd::detail::PylonSource;
#endif

#include <fstream>

using ftl::rgbd::Source;
using ftl::Configurable;
using std::string;
using ftl::rgbd::detail::StereoVideoSource;
//using ftl::rgbd::detail::NetSource;
using ftl::rgbd::detail::ImageSource;
using ftl::rgbd::detail::MiddleburySource;
using ftl::rgbd::detail::ScreenCapture;
using ftl::rgbd::capability_t;
using ftl::codecs::Channel;
//using ftl::rgbd::detail::FileSource;
using ftl::rgbd::Camera;
using ftl::rgbd::FrameCallback;


Source::Source(ftl::config::json_t &cfg) : Configurable(cfg), pose_(Eigen::Matrix4d::Identity()), net_(nullptr) {
	impl_ = nullptr;
	//params_ = {};
	stream_ = 0;
	is_dispatching = false;
	is_retrieving = false;
	reset();

	on("uri", [this](const ftl::config::Event &e) {
		LOG(INFO) << "URI change for source: " << getURI();
		reset();
	});
}

Source::Source(ftl::config::json_t &cfg, ftl::net::Universe *net) : Configurable(cfg), pose_(Eigen::Matrix4d::Identity()), net_(net) {
	impl_ = nullptr;
	//params_ = {};
	stream_ = 0;
	is_dispatching = false;
	is_retrieving = false;
	reset();

	on("uri", [this](const ftl::config::Event &e) {
		LOG(INFO) << "URI change for source: " << getURI();
		reset();
	});
}

Source::~Source() {
	if (impl_) delete impl_;
}

cv::Mat Source::cameraMatrix() const {
	cv::Mat m = (cv::Mat_<float>(3,3) << parameters().fx, 0.0, -parameters().cx, 0.0, parameters().fy, -parameters().cy, 0.0, 0.0, 1.0);
	return m;
}

static ftl::rgbd::detail::Source *createFileImpl(const ftl::URI &uri, Source *host) {
	std::string path = uri.getPath();
	// Note: This is non standard
	if (uri.getHost() == "." || uri.getHost() == "~") path = uri.getHost()+path;

	auto eix = path.find_last_of('.');

	if (eix == string::npos) {
		// Might be a directory
		if (ftl::is_directory(path)) {
			if (ftl::is_file(path + "/video.mp4")) {
				return new StereoVideoSource(host, path);
//			} else if (ftl::is_file(path + "/im0.png")) {
//				return new MiddleburySource(this, path);
			} else {
				LOG(ERROR) << "Directory is not a valid RGBD source: " << path;
			}
		} else {
			return nullptr;
		}
	} else if (ftl::is_file(path)) {
		string ext = path.substr(eix+1);

		if (ext == "ftl") {
			//ftl::rgbd::Player *reader = __createReader(path);
			//LOG(INFO) << "Playing track: " << uri.getFragment();
			//return new FileSource(this, reader, std::stoi(uri.getFragment()));
			LOG(FATAL) << "File sources not supported";
			return nullptr;
		} else if (ext == "png" || ext == "jpg") {
			return new ImageSource(host, path);
		} else if (ext == "mp4") {
			return new StereoVideoSource(host, path);
		} else {
			LOG(WARNING) << "Unrecognised file type: " << path;	
		}
	} else {
		LOG(WARNING) << "File does not exist: " << path;
	}

	return nullptr;
}

static ftl::rgbd::detail::Source *createDeviceImpl(const ftl::URI &uri, Source *host) {
	if (uri.getPathSegment(0) == "video") {
		return new StereoVideoSource(host);
	} else if (uri.getPathSegment(0) == "pylon") {
#ifdef HAVE_PYLON
		return new PylonSource(host);
#else
		LOG(ERROR) << "You did not build with 'pylon'";
#endif
	} else if (uri.getPathSegment(0) == "realsense") {
#ifdef HAVE_REALSENSE
		return new RealsenseSource(host);
#else
		LOG(ERROR) << "You do not have 'librealsense2' installed";
#endif
	} else if (uri.getPathSegment(0) == "screen") {
		return new ScreenCapture(host);
	} else {
		/*params_.width = value("width", 1280);
		params_.height = value("height", 720);
		params_.fx = value("focal", 700.0f);
		params_.fy = params_.fx;
		params_.cx = -(double)params_.width / 2.0;
		params_.cy = -(double)params_.height / 2.0;
		params_.minDepth = value("minDepth", 0.1f);
		params_.maxDepth = value("maxDepth", 20.0f);
		params_.doffs = 0;
		params_.baseline = value("baseline", 0.0f);*/
	}
	return nullptr;
}

static ftl::rgbd::detail::Source *createImplementation(const std::string &uristr, Source *host) {
	ftl::URI uri(uristr);
	if (!uri.isValid()) {
		LOG(WARNING) << "Invalid URI for source: " << uristr;
		return nullptr;
	}

	switch (uri.getScheme()) {
	case ftl::URI::SCHEME_FILE		:	return createFileImpl(uri, host);
	case ftl::URI::SCHEME_DEVICE	:	return createDeviceImpl(uri, host);
	default: break;
	}

	LOG(WARNING) << "Unrecognised source URI: " << uristr;
	return nullptr;
}

void Source::setPose(const Eigen::Matrix4d &pose) {
	pose_ = pose;
	if (impl_) impl_->setPose(pose);
}

bool Source::hasCapabilities(capability_t c) {
	return (getCapabilities() & c) == c;
}

capability_t Source::getCapabilities() const {
	if (impl_) return impl_->capabilities_;
	else return kCapMovable | kCapVideo | kCapStereo;  // FIXME: Don't assume these
}

void Source::reset() {
	UNIQUE_LOCK(mutex_,lk);
	channel_ = Channel::None;
	if (impl_) delete impl_;
	impl_ = nullptr;

	auto uristr = get<string>("uri");
	if (!uristr) return;
	impl_ = createImplementation(*uristr, this);
}

bool Source::capture(int64_t ts) {
	//timestamp_ = ts;
	if (impl_) return impl_->capture(ts);
	else return true;
}

bool Source::retrieve() {
	is_retrieving = true;
	bool status = false;
	if (impl_) status = impl_->retrieve(frames_[0]);
	is_retrieving = false;
	return status;
}

bool Source::dispatch(int64_t ts) {
	if (!callback_) return false;
	if (is_dispatching || is_retrieving) return false;
	is_dispatching = true;
	_swap();
	ftl::pool.push([this,ts](int id) {
		callback_(ts, frames_[1]);
		is_dispatching = false;
	});
	return true;
}

void Source::_swap() {
	auto tmp = std::move(frames_[0]);
	frames_[0] = std::move(frames_[1]);
	frames_[1] = std::move(tmp);
}

bool Source::setChannel(ftl::codecs::Channel c) {
	channel_ = c;
	// FIXME:(Nick) Verify channel is supported by this source...
	return true;
}

const ftl::rgbd::Camera Source::parameters(ftl::codecs::Channel chan) const {
	return (impl_) ? impl_->parameters(chan) : parameters();
}

void Source::setCallback(const FrameCallback &cb) {
	if (bool(callback_)) LOG(ERROR) << "Source already has a callback: " << getURI();
	callback_ = cb;
}

/*
 * Scale camera parameters to match resolution.
 */
Camera Camera::scaled(int width, int height) const {
	const auto &cam = *this;
	float scaleX = (float)width / (float)cam.width;
	float scaleY = (float)height / (float)cam.height;

	//CHECK( abs(scaleX - scaleY) < 0.00000001f );

	Camera newcam = cam;
	newcam.width = width;
	newcam.height = height;
	newcam.fx *= scaleX;
	newcam.fy *= scaleY;
	newcam.cx *= scaleX;
	newcam.cy *= scaleY;
	newcam.doffs *= scaleX;

	return newcam;
}

