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

#include "file_source.hpp"

#ifdef HAVE_REALSENSE
#include "realsense_source.hpp"
using ftl::rgbd::detail::RealsenseSource;
#endif

using ftl::rgbd::Source;
using ftl::Configurable;
using std::string;
using ftl::rgbd::detail::StereoVideoSource;
using ftl::rgbd::detail::NetSource;
using ftl::rgbd::detail::ImageSource;
using ftl::rgbd::detail::MiddleburySource;
using ftl::rgbd::capability_t;
using ftl::rgbd::Channel;
using ftl::rgbd::detail::FileSource;

std::map<std::string, ftl::codecs::Reader*> Source::readers__;

Source::Source(ftl::config::json_t &cfg) : Configurable(cfg), pose_(Eigen::Matrix4d::Identity()), net_(nullptr) {
	impl_ = nullptr;
	params_ = {};
	stream_ = 0;
	timestamp_ = 0;
	reset();

	on("uri", [this](const ftl::config::Event &e) {
		LOG(INFO) << "URI change for source: " << getURI();
		reset();
	});
}

Source::Source(ftl::config::json_t &cfg, ftl::net::Universe *net) : Configurable(cfg), pose_(Eigen::Matrix4d::Identity()), net_(net) {
	impl_ = nullptr;
	params_ = {};
	stream_ = 0;
	timestamp_ = 0;
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

/*void Source::customImplementation(ftl::rgbd::detail::Source *impl) {
	if (impl_) delete impl_;
	impl_ = impl;
}*/

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

		if (ext == "ftl") {
			ftl::codecs::Reader *reader = __createReader(path);
			return new FileSource(this, reader, std::stoi(uri.getFragment()));
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

ftl::codecs::Reader *Source::__createReader(const std::string &path) {
	if (readers__.find(path) != readers__.end()) {
		return readers__[path];
	}

	std::ifstream *file = new std::ifstream;
	file->open(path);

	// FIXME: This is a memory leak, must delete ifstream somewhere.

	auto *r = new ftl::codecs::Reader(*file);
	readers__[path] = r;
	r->begin();
	return r;
}

ftl::rgbd::detail::Source *Source::_createNetImpl(const ftl::URI &uri) {
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
	} else {
		params_.width = value("width", 1280);
		params_.height = value("height", 720);
		params_.fx = value("focal", 700.0f);
		params_.fy = params_.fx;
		params_.cx = -(double)params_.width / 2.0;
		params_.cy = -(double)params_.height / 2.0;
		params_.minDepth = value("minDepth", 0.1f);
		params_.maxDepth = value("maxDepth", 20.0f);
		params_.doffs = 0;
		params_.baseline = value("baseline", 0.0f);
	}
	return nullptr;
}

void Source::getFrames(cv::Mat &rgb, cv::Mat &depth) {
	if (bool(callback_)) LOG(WARNING) << "Cannot use getFrames and callback in source";
	SHARED_LOCK(mutex_,lk);
	rgb_.copyTo(rgb);
	depth_.copyTo(depth);
	//rgb = rgb_;
	//depth = depth_;

	/*cv::Mat tmp;
	tmp = rgb;
	rgb = rgb_;
	rgb_ = tmp;
	tmp = depth;
	depth = depth_;
	depth_ = tmp;*/
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
	if (!impl_ && stream_ != 0) {
		cudaSafeCall(cudaStreamSynchronize(stream_));
		if (depth_.type() == CV_32SC1) depth_.convertTo(depth_, CV_32F, 1.0f / 1000.0f);
		stream_ = 0;
		return true;
	} else if (impl_ && impl_->compute(N,B)) {
		timestamp_ = impl_->timestamp_;
		/*cv::Mat tmp;
		rgb_.create(impl_->rgb_.size(), impl_->rgb_.type());
		depth_.create(impl_->depth_.size(), impl_->depth_.type());
		tmp = rgb_;
		rgb_ = impl_->rgb_;
		impl_->rgb_ = tmp;
		tmp = depth_;
		depth_ = impl_->depth_;
		impl_->depth_ = tmp;*/

		// TODO:(Nick) Reduce buffer copies
		impl_->rgb_.copyTo(rgb_);
		impl_->depth_.copyTo(depth_);
		//rgb_ = impl_->rgb_;
		//depth_ = impl_->depth_;
		return true;
	}
	return false;
}

bool Source::thumbnail(cv::Mat &t) {
	if (!impl_ && stream_ != 0) {
		cudaSafeCall(cudaStreamSynchronize(stream_));
		if (depth_.type() == CV_32SC1) depth_.convertTo(depth_, CV_32F, 1.0f / 1000.0f);
		stream_ = 0;
		return true;
	} else if (impl_) {
		UNIQUE_LOCK(mutex_,lk);
		impl_->capture(0);
		impl_->swap();
		impl_->compute(1, 9);
		impl_->rgb_.copyTo(rgb_);
		impl_->depth_.copyTo(depth_);
	}
	if (!rgb_.empty()) {
		SHARED_LOCK(mutex_,lk);
		// Downsize and square the rgb_ image
		cv::resize(rgb_, thumb_, cv::Size(320,180));
	}
	t = thumb_;
	return !thumb_.empty();
}

bool Source::setChannel(ftl::rgbd::Channel c) {
	channel_ = c;
	// FIXME:(Nick) Verify channel is supported by this source...
	return true;
}

const ftl::rgbd::Camera Source::parameters(ftl::rgbd::Channel chan) const {
	return (impl_) ? impl_->parameters(chan) : parameters();
}

void Source::setCallback(std::function<void(int64_t, cv::Mat &, cv::Mat &)> cb) {
	if (bool(callback_)) LOG(ERROR) << "Source already has a callback: " << getURI();
	callback_ = cb;
}

void Source::addRawCallback(const std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &f) {
	UNIQUE_LOCK(mutex_,lk);
	rawcallbacks_.push_back(f);
}

void Source::removeRawCallback(const std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &f) {
	UNIQUE_LOCK(mutex_,lk);
	for (auto i=rawcallbacks_.begin(); i!=rawcallbacks_.end(); ++i) {
		if (i->target<void(*)(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)>() == f.target<void(*)(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)>()) {
			rawcallbacks_.erase(i);
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
