#include <loguru.hpp>
#include <ftl/rgbd/source.hpp>

#include "net.hpp"
#include "stereovideo.hpp"
#include "image.hpp"

#ifdef HAVE_LIBARCHIVE
#include "snapshot_source.hpp"
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
using ftl::rgbd::capability_t;

Source::Source(ftl::config::json_t &cfg) : Configurable(cfg), net_(nullptr) {
	impl_ = nullptr;
	params_ = {0};
	reset();

	on("uri", [this](const ftl::config::Event &e) {
		LOG(INFO) << "URI change for source: " << getURI();
		reset();
	});
}

Source::Source(ftl::config::json_t &cfg, ftl::net::Universe *net) : Configurable(cfg), net_(net) {
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
			return new StereoVideoSource(this);
		} else {
			return nullptr;
		}
	} else if (ftl::is_file(path)) {
		string ext = path.substr(eix+1);

		if (ext == "png" || ext == "jpg") {
			return new ImageSource(this, path);
		} else if (ext == "mp4") {
			return new StereoVideoSource(this, path);
		} else if (ext == "tar") {
			return nullptr;
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
	}
	return nullptr;
}

void Source::getFrames(cv::Mat &rgb, cv::Mat &depth) {
	shared_lock<shared_mutex> lk(mutex_);
	rgb_.copyTo(rgb);
	depth_.copyTo(depth);
}

Eigen::Vector4f Source::point(uint ux, uint uy) {
	const auto &params = parameters();
	const float x = ((float)ux-(float)params.cx) / (float)params.fx;
	const float y = ((float)uy-(float)params.cy) / (float)params.fy;

	shared_lock<shared_mutex> lk(mutex_);
	const float depth = depth_.at<float>(uy,ux);
	return Eigen::Vector4f(x*depth,y*depth,depth,1.0);
}

void Source::setPose(const Eigen::Matrix4f &pose) {
	pose_ = pose;
	if (impl_) impl_->setPose(pose);
}

const Eigen::Matrix4f &Source::getPose() const {
	return pose_;
}

bool Source::hasCapability(capability_t) {
	return false;
}

void Source::reset() {
	unique_lock<shared_mutex> lk(mutex_);
	if (impl_) delete impl_;
	impl_ = _createImplementation();
}

bool Source::grab() {
	unique_lock<shared_mutex> lk(mutex_);
	if (impl_ && impl_->grab()) {
		impl_->rgb_.copyTo(rgb_);
		impl_->depth_.copyTo(depth_);
		return true;
	}
	return false;
}
