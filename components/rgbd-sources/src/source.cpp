#include <loguru.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/threads.hpp>

#include "net.hpp"
#include "stereovideo.hpp"
#include "image.hpp"
#include "middlebury_source.hpp"

#ifdef HAVE_LIBARCHIVE
#include <ftl/rgbd/snapshot.hpp>
#include "snapshot_source.hpp"
#endif

#ifdef HAVE_REALSENSE
#include "realsense_source.hpp"
using ftl::rgbd::detail::RealsenseSource;
#endif

using ftl::rgbd::Source;
using ftl::Configurable;
using std::string;
using std::shared_mutex;
using std::unique_lock;
using std::shared_lock;
using ftl::rgbd::detail::StereoVideoSource;
using ftl::rgbd::detail::NetSource;
using ftl::rgbd::detail::ImageSource;
using ftl::rgbd::detail::MiddleburySource;
using ftl::rgbd::capability_t;

Source::Source(ftl::config::json_t &cfg) : Configurable(cfg), pose_(Eigen::Matrix4d::Identity()), net_(nullptr) {
	impl_ = nullptr;
	params_ = {0};
	reset();

	on("uri", [this](const ftl::config::Event &e) {
		LOG(INFO) << "URI change for source: " << getURI();
		reset();
	});
}

Source::Source(ftl::config::json_t &cfg, ftl::net::Universe *net) : Configurable(cfg), pose_(Eigen::Matrix4d::Identity()), net_(net) {
	impl_ = nullptr;
	params_ = {0};
	reset();

	on("uri", [this](const ftl::config::Event &e) {
		LOG(INFO) << "URI change for source: " << getURI();
		reset();
	});
}

Source::~Source() {

}

cv::Mat Source::cameraMatrix() const {
	cv::Mat m = (cv::Mat_<float>(3,3) << parameters().fx, 0.0, -parameters().cx, 0.0, parameters().fy, -parameters().cy, 0.0, 0.0, 1.0);
	return m;
}

void Source::customImplementation(ftl::rgbd::detail::Source *impl) {
	if (impl_) delete impl_;
	impl_ = impl;
}

ftl::rgbd::detail::Source *Source::_createImplementation() {
	auto uristr = get<string>("uri");
	if (!uristr) {
		//LOG(WARNING) << "Missing URI for source";
		return nullptr;
	}

	ftl::URI uri(*uristr);
	if (!uri.isValid()) {
		LOG(WARNING) << "Invalid URI for source: " << *uristr;
		return nullptr;
	}

	switch (uri.getScheme()) {
	case ftl::URI::SCHEME_FILE		:	return _createFileImpl(uri);
	case ftl::URI::SCHEME_FTL		:	return _createNetImpl(uri);
	case ftl::URI::SCHEME_DEVICE	:	return _createDeviceImpl(uri);
	default: break;
	}

	LOG(WARNING) << "Unrecognised source URI: " << *uristr;
	return nullptr;
}

ftl::rgbd::detail::Source *Source::_createFileImpl(const ftl::URI &uri) {
	std::string path = uri.getPath();
	// Note: This is non standard
	if (uri.getHost() == "." || uri.getHost() == "~") path = uri.getHost()+path;

	auto eix = path.find_last_of('.');

	if (eix == string::npos) {
		// Might be a directory
		if (ftl::is_directory(path)) {
			if (ftl::is_file(path + "/video.mp4")) {
				return new StereoVideoSource(this, path);
			} else if (ftl::is_file(path + "/im0.png")) {
				return new MiddleburySource(this, path);
			} else {
				LOG(ERROR) << "Directory is not a valid RGBD source: " << path;
			}
		} else {
			return nullptr;
		}
	} else if (ftl::is_file(path)) {
		string ext = path.substr(eix+1);

		if (ext == "png" || ext == "jpg") {
			return new ImageSource(this, path);
		} else if (ext == "mp4") {
			return new StereoVideoSource(this, path);
		} else if (ext == "tar" || ext == "gz") {
#ifdef HAVE_LIBARCHIVE
			ftl::rgbd::SnapshotReader reader(path);
			return new ftl::rgbd::detail::SnapshotSource(this, reader, std::to_string(value("index", 0)));  // TODO Get ID from config
#else
			LOG(ERROR) << "Cannot read snapshots, libarchive not installed";
			return nullptr;
#endif  // HAVE_LIBARCHIVE
		} else {
			LOG(WARNING) << "Unrecognised file type: " << path;	
		}
	} else {
		LOG(WARNING) << "File does not exist: " << path;
	}

	return nullptr;
}

ftl::rgbd::detail::Source *Source::_createNetImpl(const ftl::URI &uri) {
	LOG(INFO) << "MAKE NET SOURCE";
	return new NetSource(this);
}

ftl::rgbd::detail::Source *Source::_createDeviceImpl(const ftl::URI &uri) {
	if (uri.getPathSegment(0) == "video") {
		return new StereoVideoSource(this);
	} else if (uri.getPathSegment(0) == "realsense") {
#ifdef HAVE_REALSENSE
		return new RealsenseSource(this);
#else
		LOG(ERROR) << "You do not have 'librealsense2' installed";
#endif
	}
	return nullptr;
}

void Source::getFrames(cv::Mat &rgb, cv::Mat &depth) {
	SHARED_LOCK(mutex_,lk);
	//rgb_.copyTo(rgb);
	//depth_.copyTo(depth);
	rgb = rgb_;
	depth = depth_;
}

Eigen::Vector4d Source::point(uint ux, uint uy) {
	const auto &params = parameters();
	const double x = ((double)ux+params.cx) / params.fx;
	const double y = ((double)uy+params.cy) / params.fy;

	SHARED_LOCK(mutex_,lk);
	const double depth = depth_.at<float>(uy,ux);
	return Eigen::Vector4d(x*depth,y*depth,depth,1.0);
}

Eigen::Vector4d Source::point(uint ux, uint uy, double d) {
	const auto &params = parameters();
	const double x = ((double)ux+params.cx) / params.fx;
	const double y = ((double)uy+params.cy) / params.fy;
	return Eigen::Vector4d(x*d,y*d,d,1.0);
}

Eigen::Vector2i Source::point(const Eigen::Vector4d &p) {
	const auto &params = parameters();
	double x = p[0] / p[2];
	double y = p[1] / p[2];
	x *= params.fx;
	y *= params.fy;
	return Eigen::Vector2i((int)(x - params.cx), (int)(y - params.cy));
}

void Source::setPose(const Eigen::Matrix4d &pose) {
	pose_ = pose;
	if (impl_) impl_->setPose(pose);
}

const Eigen::Matrix4d &Source::getPose() const {
	return pose_;
}

bool Source::hasCapabilities(capability_t c) {
	return getCapabilities() & c == c;
}

capability_t Source::getCapabilities() const {
	if (impl_) return impl_->capabilities_;
	else return 0;
}

void Source::reset() {
	UNIQUE_LOCK(mutex_,lk);
	if (impl_) delete impl_;
	impl_ = _createImplementation();
}

bool Source::grab() {
	UNIQUE_LOCK(mutex_,lk);
	if (impl_ && impl_->grab()) {
		impl_->rgb_.copyTo(rgb_);
		impl_->depth_.copyTo(depth_);
		return true;
	}
	return false;
}
