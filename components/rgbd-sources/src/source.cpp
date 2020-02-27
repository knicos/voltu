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
using ftl::rgbd::RawCallback;
using ftl::rgbd::FrameCallback;

std::map<std::string, ftl::rgbd::Player*> Source::readers__;

Source::Source(ftl::config::json_t &cfg) : Configurable(cfg), pose_(Eigen::Matrix4d::Identity()), net_(nullptr) {
	impl_ = nullptr;
	//params_ = {};
	stream_ = 0;
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
			return new ImageSource(this, path);
		} else if (ext == "mp4") {
			return new StereoVideoSource(this, path);
		} else if (ext == "tar" || ext == "gz") {
#ifdef HAVE_LIBARCHIVE
			ftl::rgbd::SnapshotReader reader(path);
			auto snapshot = reader.readArchive();
			return new ftl::rgbd::detail::SnapshotSource(this, snapshot, value("index", std::string("0")));  // TODO: Use URI fragment
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

/*ftl::rgbd::Player *Source::__createReader(const std::string &path) {
	if (readers__.find(path) != readers__.end()) {
		return readers__[path];
	}

	std::ifstream *file = new std::ifstream;
	file->open(path);

	// FIXME: This is a memory leak, must delete ifstream somewhere.

	auto *r = new ftl::rgbd::Player(*file);
	readers__[path] = r;
	r->begin();
	return r;
}*/

ftl::rgbd::detail::Source *Source::_createNetImpl(const ftl::URI &uri) {
	LOG(FATAL) << "Net sources no longer supported";
	//return new NetSource(this);
	return nullptr;
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
	} else if (uri.getPathSegment(0) == "screen") {
		return new ScreenCapture(this);
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

void Source::getFrames(cv::Mat &rgb, cv::Mat &depth) {
	if (bool(callback_)) LOG(WARNING) << "Cannot use getFrames and callback in source";
	SHARED_LOCK(mutex_,lk);
	//rgb_.copyTo(rgb);
	//depth_.copyTo(depth);
	//rgb = rgb_;
	//depth = depth_;
}


void Source::setPose(const Eigen::Matrix4d &pose) {
	pose_ = pose;
	if (impl_) impl_->setPose(pose);
}

const Eigen::Matrix4d &Source::getPose() const {
	return pose_;
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
	impl_ = _createImplementation();
}

bool Source::capture(int64_t ts) {
	//timestamp_ = ts;
	if (impl_) return impl_->capture(ts);
	else return true;
}

bool Source::retrieve() {
	if (impl_) return impl_->retrieve();
	else return true;
}

bool Source::compute(int N, int B) {
	UNIQUE_LOCK(mutex_,lk);
	return impl_ && impl_->compute(N,B);
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

void Source::addRawCallback(const RawCallback &f) {
	UNIQUE_LOCK(mutex_,lk);
	rawcallbacks_.push_back(f);
}

void Source::removeRawCallback(const std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &f) {
	UNIQUE_LOCK(mutex_,lk);
	for (auto i=rawcallbacks_.begin(); i!=rawcallbacks_.end(); ++i) {
		const auto targ = (*i).target<void(*)(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)>();
		if (targ && targ == f.target<void(*)(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)>()) {
			rawcallbacks_.erase(i);
			LOG(INFO) << "Removing RAW callback";
			return;
		}
	}
}

void Source::notifyRaw(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	SHARED_LOCK(mutex_,lk);

	for (auto &i : rawcallbacks_) {
		i(this, spkt, pkt);
	}
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

void Source::notify(int64_t ts, ftl::rgbd::Frame &f) {
	//if (impl_) f.setOrigin(&impl_->state_);
	if (callback_) callback_(ts, f);
}

void Source::inject(const Eigen::Matrix4d &pose) {
	ftl::codecs::StreamPacket spkt;
	ftl::codecs::Packet pkt;

	spkt.timestamp = impl_->timestamp_;
	spkt.frame_number = 0;
	spkt.channel = Channel::Pose;
	spkt.streamID = 0;
	pkt.codec = ftl::codecs::codec_t::MSGPACK;
	pkt.definition = ftl::codecs::definition_t::Any;
	pkt.bitrate = 0;
	pkt.frame_count = 1;
	pkt.flags = 0;

	std::vector<double> data(pose.data(), pose.data() + 4*4*sizeof(double));
	VectorBuffer buf(pkt.data);
	msgpack::pack(buf, data);

	notifyRaw(spkt, pkt);
}
