
#include <ftl/rgbd/frame.hpp>
#include <ftl/calibration/structures.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::codecs::Channels;
using ftl::codecs::Channel;
using ftl::rgbd::VideoFrame;

VideoFrame::VideoFrame(const VideoFrame &f) {
	gpu = f.gpu;
	host = f.host;
	isgpu = f.isgpu;
	validhost = f.validhost;
}

VideoFrame &VideoFrame::operator=(const VideoFrame &f) {
	gpu = f.gpu;
	host = f.host;
	isgpu = f.isgpu;
	validhost = f.validhost;
	return *this;
}


/*cv::Mat &Frame::fastDownload(ftl::codecs::Channel c, cv::cuda::Stream stream) {
	if (hasChannel(c)) {
		auto &data = getData(static_cast<Channel>(c));
		if (!data.isgpu) return data.host;

		if (data.validhost && !data.host.empty()) return data.host;

		// TODO: Perhaps allocated page locked here?
		data.gpu.download(data.host, stream);
		data.validhost = true;
		return data.host;
	}
	throw FTL_Error("Fast download channel does not exist: " << (int)c);
}*/

cv::Mat &VideoFrame::createCPU() {
	isgpu = false;
	return host;
}

cv::cuda::GpuMat &VideoFrame::createGPU() {
	isgpu = true;
	validhost = false;
	return gpu;
}

cv::Mat &VideoFrame::createCPU(const ftl::rgbd::FormatBase &f) {
	if (!f.empty()) {
		host.create(f.size(), f.cvType);
	}
	isgpu = false;

	return host;
}

cv::cuda::GpuMat &VideoFrame::createGPU(const ftl::rgbd::FormatBase &f) {
	if (!f.empty()) {
		gpu.create(f.size(), f.cvType);
	}
	isgpu = true;
	validhost = false;

	return gpu;
}

const cv::Mat &VideoFrame::getCPU() const {
	if (!validhost) {
		// TODO: Use stream and page locked mem.
		gpu.download(host);
		validhost = true;
	}
	return host;
}

const cv::cuda::GpuMat &VideoFrame::getGPU() const {
	// TODO: Upload?
	return gpu;
}

cv::Mat &VideoFrame::setCPU() {
	validhost = true;
	return host;
}

cv::cuda::GpuMat &VideoFrame::setGPU() {
	validhost = false;
	return gpu;
}

void ftl::rgbd::Frame::upload(ftl::codecs::Channel c) {
	auto &vframe = set<VideoFrame>(c);
	const auto &cpumat = vframe.getCPU();
	LOG(WARNING) << "Sync Upload: " << int(c);
	vframe.createGPU().upload(cpumat);
}

bool ftl::rgbd::Frame::isGPU(ftl::codecs::Channel c) const {
	const auto &vframe = get<VideoFrame>(c);
	return vframe.isGPU();
}

bool ftl::rgbd::Frame::hasOpenGL(ftl::codecs::Channel c) const {
	const auto &vframe = get<VideoFrame>(c);
	return vframe.hasOpenGL();
}

unsigned int ftl::rgbd::Frame::getOpenGL(ftl::codecs::Channel c) const {
	const auto &vframe = get<VideoFrame>(c);
	return vframe.getOpenGL();
}

cv::Size ftl::rgbd::Frame::getSize(ftl::codecs::Channel c) const {
	if (hasChannel(c)) {
		const auto &f = get<VideoFrame>(c);
		if (f.isGPU()) {
			return f.getGPU().size();
		} else {
			return f.getCPU().size();
		}
	} else {
		//throw FTL_Error("Channel does not exists: " << int(c));
		return cv::Size(0,0);
	}
}

const ftl::rgbd::Camera &ftl::rgbd::Frame::getLeftCamera() const {
	return std::get<0>(this->get<std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, int>>(ftl::codecs::Channel::Calibration));
}

const ftl::rgbd::Camera &ftl::rgbd::Frame::getRightCamera() const {
	return std::get<0>(this->get<std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, int>>(ftl::codecs::Channel::Calibration2));
}

const Eigen::Matrix4d &ftl::rgbd::Frame::getPose() const {
	return this->get<Eigen::Matrix4d>(ftl::codecs::Channel::Pose);
}

ftl::rgbd::Camera &ftl::rgbd::Frame::setLeft() {
	return std::get<0>(this->create<std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, int>>(ftl::codecs::Channel::Calibration));
}

ftl::rgbd::Camera &ftl::rgbd::Frame::setRight() {
	return std::get<0>(this->create<std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, int>>(ftl::codecs::Channel::Calibration2));
}

Eigen::Matrix4d &ftl::rgbd::Frame::setPose() {
	return this->create<Eigen::Matrix4d>(ftl::codecs::Channel::Pose);
}

const ftl::calibration::CalibrationData& ftl::rgbd::Frame::getCalibration() const {
	return this->get<ftl::calibration::CalibrationData>(Channel::CalibrationData);
}

ftl::calibration::CalibrationData& ftl::rgbd::Frame::setCalibration() {
	return this->create<ftl::calibration::CalibrationData>(Channel::CalibrationData);
}

std::string ftl::rgbd::Frame::serial() const {
	if (hasChannel(Channel::MetaData)) {
		const auto &meta = get<std::map<std::string,std::string>>(Channel::MetaData);
		auto i = meta.find("serial");
		if (i != meta.end()) return i->second;
	}
	return "";
}

std::string ftl::rgbd::Frame::device() const {
	if (hasChannel(Channel::MetaData)) {
		const auto &meta = get<std::map<std::string,std::string>>(Channel::MetaData);
		auto i = meta.find("device");
		if (i != meta.end()) return i->second;
	}
	return "";
}

const std::unordered_set<ftl::rgbd::Capability> &ftl::rgbd::Frame::capabilities() const {
	return get<std::unordered_set<ftl::rgbd::Capability>>(Channel::Capabilities);
}

bool ftl::rgbd::Frame::hasCapability(ftl::rgbd::Capability c) const {
	if (hasChannel(Channel::Capabilities)) {
		const auto &cap = get<std::unordered_set<ftl::rgbd::Capability>>(Channel::Capabilities);
		return cap.count(c) > 0;
	}
	return false;
}


template <>
cv::Mat &ftl::data::Frame::create<cv::Mat, 0>(ftl::codecs::Channel c) {
	return create<ftl::rgbd::VideoFrame>(c).createCPU();
}

template <>
cv::cuda::GpuMat &ftl::data::Frame::create<cv::cuda::GpuMat, 0>(ftl::codecs::Channel c) {
	return create<ftl::rgbd::VideoFrame>(c).createGPU();
}

template <>
const cv::Mat &ftl::data::Frame::get<cv::Mat>(ftl::codecs::Channel c) const {
	return get<ftl::rgbd::VideoFrame>(c).getCPU();
}

template <>
const cv::cuda::GpuMat &ftl::data::Frame::get<cv::cuda::GpuMat>(ftl::codecs::Channel c) const {
	return get<ftl::rgbd::VideoFrame>(c).getGPU();
}

template <>
cv::Mat &ftl::data::Frame::set<cv::Mat, 0>(ftl::codecs::Channel c) {
	return set<ftl::rgbd::VideoFrame>(c).setCPU();
}

template <>
cv::cuda::GpuMat &ftl::data::Frame::set<cv::cuda::GpuMat, 0>(ftl::codecs::Channel c) {
	return set<ftl::rgbd::VideoFrame>(c).setGPU();
}

