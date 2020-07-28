#include <loguru.hpp>
#include <ftl/rgbd/source.hpp>
#include "basesource.hpp"
#include <ftl/threads.hpp>

//#include "sources/net/net.hpp"
#include "sources/stereovideo/stereovideo.hpp"
#include "sources/image/image.hpp"
#include "sources/middlebury/middlebury_source.hpp"
#include "sources/screencapture/screencapture.hpp"

//#include "sources/ftlfile/file_source.hpp"

#ifdef HAVE_REALSENSE
#include "sources/realsense/realsense_source.hpp"
using ftl::rgbd::detail::RealsenseSource;
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
using ftl::codecs::Channel;
using ftl::rgbd::Camera;

Source::Source(ftl::config::json_t &cfg) : Configurable(cfg) {
	impl_ = nullptr;
	stream_ = 0;
	is_retrieving = false;
	reset();

	on("uri", [this]() {
		LOG(INFO) << "URI change for source: " << getURI();
		reset();
	});
}

Source::~Source() {
	if (impl_) delete impl_;
}

bool Source::isReady() { return (impl_) ? impl_->isReady() : false; }

static ftl::rgbd::BaseSourceImpl *createFileImpl(const ftl::URI &uri, Source *host) {
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

static ftl::rgbd::BaseSourceImpl *createDeviceImpl(const ftl::URI &uri, Source *host) {
	if (uri.getPathSegment(0) == "stereo" || uri.getPathSegment(0) == "video" || uri.getPathSegment(0) == "camera" || uri.getPathSegment(0) == "pylon") {
		return new StereoVideoSource(host);
	} else if (uri.getPathSegment(0) == "realsense") {
#ifdef HAVE_REALSENSE
		return new RealsenseSource(host);
#else
		LOG(ERROR) << "You do not have 'librealsense2' installed";
#endif
	} else if (uri.getPathSegment(0) == "screen") {
		return new ScreenCapture(host);
	}
	return nullptr;
}

static ftl::rgbd::BaseSourceImpl *createImplementation(const std::string &uristr, Source *host) {
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

void Source::reset() {
	UNIQUE_LOCK(mutex_,lk);
	if (impl_) delete impl_;
	impl_ = nullptr;

	auto uristr = get<string>("uri");
	if (!uristr) return;

	ftl::URI uri(*uristr);

	restore(uri.getBaseURI(), {
		"min_depth",
		"max_depth",
		"name",
		"offset_z",
		"size",
		"focal",
		"device_left",
		"enable_touch",
		"feed",
		"pipeline"
	});

	uri.to_json(getConfig());

	impl_ = createImplementation(*uristr, this);
}

bool Source::capture(int64_t ts) {
	if (impl_) return impl_->capture(ts);
	else return true;
}

bool Source::retrieve(ftl::data::Frame &f) {
	if (is_retrieving) return false;
	is_retrieving = true;
	bool status = false;
	if (impl_) status = impl_->retrieve(f.cast<ftl::rgbd::Frame>());
	is_retrieving = false;
	return status;
}

bool Source::supports(const std::string &puri) {
	ftl::URI uri(puri);

	if (uri.getPathSegment(0) == "video") {
		return StereoVideoSource::supported(uri.getPathSegment(0));
	} else if (uri.getPathSegment(0) == "camera" || uri.getPathSegment(0) == "stereo") {
		return StereoVideoSource::supported(uri.getPathSegment(0));
	} else if (uri.getPathSegment(0) == "pylon") {
		return StereoVideoSource::supported(uri.getPathSegment(0));
	} else if (uri.getPathSegment(0) == "realsense") {
		#ifdef HAVE_REALSENSE
		return RealsenseSource::supported();
		#endif
	} else if (uri.getPathSegment(0) == "screen") {
		#ifndef WIN32
		return true;
		#endif
	}

	return false;
}


