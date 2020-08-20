#pragma once
#ifndef _FTL_RGBD_FRAME_HPP_
#define _FTL_RGBD_FRAME_HPP_

#include <ftl/configuration.hpp>
#include <ftl/exception.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>

#include <ftl/data/new_frame.hpp>

#include <ftl/codecs/channels.hpp>
#include <ftl/rgbd/format.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/codecs/codecs.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/utility/vectorbuffer.hpp>
#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/capabilities.hpp>

#include <type_traits>
#include <array>
#include <list>

#include <Eigen/Eigen>

namespace ftl {
namespace calibration {
struct CalibrationData;
}

namespace rgbd {

//typedef ftl::data::Frame Frame;

/*inline const ftl::rgbd::Camera &getLeftCamera(const Frame &f) { return f.get<ftl::rgbd::Camera>(ftl::codecs::Channel::Calibration); }
inline const ftl::rgbd::Camera &getRightCamera(const Frame &f) { return f.get<ftl::rgbd::Camera>(ftl::codecs::Channel::Calibration2); }
inline const ftl::rgbd::Camera &getLeft(const Frame &f) { return f.get<ftl::rgbd::Camera>(ftl::codecs::Channel::Calibration); }
inline const ftl::rgbd::Camera &getRight(const Frame &f) { return f.get<ftl::rgbd::Camera>(ftl::codecs::Channel::Calibration2); }
inline const Eigen::Matrix4d &getPose(const Frame &f) { return f.get<Eigen::Matrix4d>(ftl::codecs::Channel::Pose); }*/

class VideoFrame {
	public:
	VideoFrame() {}

	// Manually add copy constructor since default is removed
	VideoFrame(const VideoFrame &);
	VideoFrame &operator=(const VideoFrame &);

	template <typename T>
	ftl::cuda::TextureObject<T> &createTexture(const ftl::rgbd::Format<T> &f, bool interpolated);

	template <typename T>
	ftl::cuda::TextureObject<T> &createTexture(bool interpolated=false) const;

	cv::cuda::GpuMat &createGPU();
	cv::cuda::GpuMat &createGPU(const ftl::rgbd::FormatBase &f);

	template <typename T> ftl::cuda::TextureObject<T> &getTexture(ftl::codecs::Channel) const;

	cv::Mat &createCPU();
	cv::Mat &createCPU(const ftl::rgbd::FormatBase &f);

	const cv::Mat &getCPU() const;
	const cv::cuda::GpuMat &getGPU() const;

	/// gets cv::Mat for
	cv::Mat &setCPU();
	cv::cuda::GpuMat &setGPU();

	inline bool isGPU() const { return isgpu; };

	inline bool hasOpenGL() const { return opengl_id != 0; }
	inline void setOpenGL(unsigned int id) { opengl_id = id; }
	inline unsigned int getOpenGL() const { return opengl_id; }


	private:
	mutable ftl::cuda::TextureObjectBase tex;
	cv::cuda::GpuMat gpu;
	mutable cv::Mat host;
	unsigned int opengl_id=0;
	bool isgpu=false;
	mutable bool validhost=false;
};

class Frame : public ftl::data::Frame {
	public:
	const ftl::rgbd::Camera &getLeftCamera() const;
	const ftl::rgbd::Camera &getRightCamera() const;
	inline const ftl::rgbd::Camera &getLeft() const { return getLeftCamera(); }
	inline const ftl::rgbd::Camera &getRight() const { return getRightCamera(); }
	const Eigen::Matrix4d &getPose() const;
	ftl::rgbd::Camera &setLeft();
	ftl::rgbd::Camera &setRight();
	Eigen::Matrix4d &setPose();

	cv::Size getSize(ftl::codecs::Channel c=ftl::codecs::Channel::Left) const;

	ftl::calibration::CalibrationData& setCalibration();
	const ftl::calibration::CalibrationData& getCalibration() const;

	std::string serial() const;
	std::string device() const;

	/** Note, this throws exception if channel is missing */
	const std::unordered_set<ftl::rgbd::Capability> &capabilities() const;

	/** Does not throw exception */
	bool hasCapability(ftl::rgbd::Capability) const;

	inline bool isLive() const { return hasCapability(ftl::rgbd::Capability::LIVE); }
	inline bool isVirtual() const { return hasCapability(ftl::rgbd::Capability::VIRTUAL); }
	inline bool isMovable() const { return hasCapability(ftl::rgbd::Capability::MOVABLE); }
	inline bool isTouchable() const { return hasCapability(ftl::rgbd::Capability::TOUCH); }
	inline bool isVR() const { return hasCapability(ftl::rgbd::Capability::VR); }
	inline bool is360() const { return hasCapability(ftl::rgbd::Capability::EQUI_RECT); }
	inline bool isSideBySideStereo() const { return hasCapability(ftl::rgbd::Capability::STEREO); }

	void upload(ftl::codecs::Channel c);

	bool isGPU(ftl::codecs::Channel c) const;
	bool hasOpenGL(ftl::codecs::Channel c) const;
	unsigned int getOpenGL(ftl::codecs::Channel c) const;

	template <typename T>
	ftl::cuda::TextureObject<T> &getTexture(ftl::codecs::Channel c) { return this->get<VideoFrame>(c).getTexture<T>(c); }

	template <typename T>
	ftl::cuda::TextureObject<T> &createTexture(ftl::codecs::Channel c, bool interpolated=false) { return this->get<VideoFrame>(c).createTexture<T>(interpolated); }

	template <typename T>
	ftl::cuda::TextureObject<T> &createTexture(ftl::codecs::Channel c, const ftl::rgbd::Format<T> &fmt, bool interpolated=false) { return this->create<VideoFrame>(c).createTexture<T>(fmt, interpolated); }

};


template <typename T>
ftl::cuda::TextureObject<T> &VideoFrame::getTexture(ftl::codecs::Channel c) const {
	if (!isgpu) throw FTL_Error("Texture channel is not on GPU");

	if (tex.cvType() != ftl::traits::OpenCVType<T>::value || tex.width() != static_cast<size_t>(gpu.cols) || tex.height() != static_cast<size_t>(gpu.rows) || gpu.type() != tex.cvType()) {
		throw FTL_Error("Texture has not been created properly " << int(c));
	}

	return ftl::cuda::TextureObject<T>::cast(tex);
}

template <typename T>
ftl::cuda::TextureObject<T> &VideoFrame::createTexture(const ftl::rgbd::Format<T> &f, bool interpolated) {
	createGPU();

	if (f.empty()) {
		throw FTL_Error("createTexture needs a non-empty format");
	} else {
		gpu.create(f.size(), f.cvType);
	}

	if (gpu.type() != ftl::traits::OpenCVType<T>::value) {
		throw FTL_Error("Texture type mismatch: " << gpu.type() << " != " << ftl::traits::OpenCVType<T>::value);
	}

	// TODO: Check tex cvType

	if (tex.devicePtr() == nullptr) {
		//LOG(INFO) << "Creating texture object";
		tex = ftl::cuda::TextureObject<T>(gpu, interpolated);
	} else if (tex.cvType() != ftl::traits::OpenCVType<T>::value || tex.width() != static_cast<size_t>(gpu.cols) || tex.height() != static_cast<size_t>(gpu.rows)) {
		//LOG(INFO) << "Recreating texture object for '" << ftl::codecs::name(c) << "'";
		tex.free();
		tex = ftl::cuda::TextureObject<T>(gpu, interpolated);
	}

	return ftl::cuda::TextureObject<T>::cast(tex);
}

template <typename T>
ftl::cuda::TextureObject<T> &VideoFrame::createTexture(bool interpolated) const {
	if (!isgpu && !host.empty()) {
		//gpu.create(host.size(), host.type());
		// TODO: Should this upload to GPU or not?
		//gpu_ += c;
		throw FTL_Error("Cannot create a texture on a host frame");
	} else if (!isgpu || (isgpu && gpu.empty())) {
		throw FTL_Error("createTexture needs a format if no memory is allocated");
	}

	if (gpu.type() != ftl::traits::OpenCVType<T>::value) {
		throw FTL_Error("Texture type mismatch: " << gpu.type() << " != " << ftl::traits::OpenCVType<T>::value);
	}

	// TODO: Check tex cvType

	if (tex.devicePtr() == nullptr) {
		//LOG(INFO) << "Creating texture object";
		tex = ftl::cuda::TextureObject<T>(gpu, interpolated);
	} else if (tex.cvType() != ftl::traits::OpenCVType<T>::value || tex.width() != static_cast<size_t>(gpu.cols) || tex.height() != static_cast<size_t>(gpu.rows) || tex.devicePtr() != gpu.data) {
		//LOG(INFO) << "Recreating texture object for '" << ftl::codecs::name(c) << "'.";
		tex.free();
		tex = ftl::cuda::TextureObject<T>(gpu, interpolated);
	}

	return ftl::cuda::TextureObject<T>::cast(tex);
}

}
}

template <>
cv::Mat &ftl::data::Frame::create<cv::Mat, 0>(ftl::codecs::Channel c);

template <>
cv::cuda::GpuMat &ftl::data::Frame::create<cv::cuda::GpuMat, 0>(ftl::codecs::Channel c);

template <>
const cv::Mat &ftl::data::Frame::get<cv::Mat>(ftl::codecs::Channel c) const;

template <>
const cv::cuda::GpuMat &ftl::data::Frame::get<cv::cuda::GpuMat>(ftl::codecs::Channel c) const;

template <>
cv::Mat &ftl::data::Frame::set<cv::Mat, 0>(ftl::codecs::Channel c);

template <>
cv::cuda::GpuMat &ftl::data::Frame::set<cv::cuda::GpuMat, 0>(ftl::codecs::Channel c);

template <>
inline bool ftl::data::make_type<ftl::rgbd::VideoFrame>() {
	return false;
}

template <>
inline bool ftl::data::decode_type<ftl::rgbd::VideoFrame>(std::any &a, const std::vector<uint8_t> &data) {
	return false;
}

#endif // _FTL_RGBD_FRAME_HPP_
