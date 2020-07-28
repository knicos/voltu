#include <ftl/utility/gltexture.hpp>

#include <nanogui/opengl.h>
#include <loguru.hpp>

#include <ftl/cuda_common.hpp>

#include <cuda_gl_interop.h>
#include <opencv2/core/cuda_stream_accessor.hpp>

#include <ftl/exception.hpp>

void log_error() {
	auto err = glGetError();
	if (err != 0) LOG(ERROR) << "OpenGL Texture error: " << err;
}

using ftl::utility::GLTexture;

GLTexture::GLTexture() {
	glid_ = std::numeric_limits<unsigned int>::max();
	glbuf_ = std::numeric_limits<unsigned int>::max();
	cuda_res_ = nullptr;
	width_ = 0;
	height_ = 0;
	type_ = Type::RGBA;
}

GLTexture::~GLTexture() {
	free();  // Note: Do not simply remove this...
}

void GLTexture::make(int width, int height, Type type) {
	if (width != width_ || height != height_ || type_ != type) {
		free();
	}

	static constexpr int ALIGNMENT = 128;

	width_ = width;
	height_ = height;
	stride_ = ((width*4) % ALIGNMENT != 0) ?
		((width*4) + (ALIGNMENT - ((width*4) % ALIGNMENT))) / 4:
		width;

	type_ = type;

	if (width == 0 || height == 0) {
		throw FTL_Error("Invalid texture size");
	}

	if (glid_ == std::numeric_limits<unsigned int>::max()) {
		glGenTextures(1, &glid_); log_error();
		glBindTexture(GL_TEXTURE_2D, glid_); log_error();
		glPixelStorei(GL_UNPACK_ROW_LENGTH, stride_); log_error();

		if (type_ == Type::BGRA) {
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_BGRA, GL_UNSIGNED_BYTE, nullptr);
		} else if (type_ == Type::Float) {
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_BGRA, GL_UNSIGNED_BYTE, nullptr);
		}
		log_error();

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		log_error();

		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

		glBindTexture(GL_TEXTURE_2D, 0);

		glGenBuffers(1, &glbuf_);
		// Make this the current UNPACK buffer (OpenGL is state-based)
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, glbuf_);
		// Allocate data for the buffer. 4-channel 8-bit image or 1-channel float
		glBufferData(GL_PIXEL_UNPACK_BUFFER, stride_ * height * 4, NULL, GL_DYNAMIC_COPY);

		cudaSafeCall(cudaGraphicsGLRegisterBuffer(&cuda_res_, glbuf_, cudaGraphicsRegisterFlagsWriteDiscard));
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		log_error();
	}
}

void GLTexture::free() {
	if (glid_ != std::numeric_limits<unsigned int>::max()) {
		glDeleteTextures(1, &glid_);
		glid_ = std::numeric_limits<unsigned int>::max();
	}

	if (glbuf_ != std::numeric_limits<unsigned int>::max()) {
		cudaSafeCall(cudaGraphicsUnregisterResource( cuda_res_ ));
		cuda_res_ = nullptr;
		glDeleteBuffers(1, &glbuf_);
		glbuf_ = std::numeric_limits<unsigned int>::max();
	}
}

cv::cuda::GpuMat GLTexture::map(cudaStream_t stream) {
	mtx_.lock();
	void *devptr;
	size_t size;
	cudaSafeCall(cudaGraphicsMapResources(1, &cuda_res_, stream));
	cudaSafeCall(cudaGraphicsResourceGetMappedPointer(&devptr, &size, cuda_res_));
	return cv::cuda::GpuMat(height_, width_, (type_ == Type::BGRA) ? CV_8UC4 : CV_32F, devptr, stride_*4);
}

void GLTexture::unmap(cudaStream_t stream) {
	// note: code must not throw, otherwise mtx_.unlock() does not happen

	cudaSafeCall(cudaGraphicsUnmapResources(1, &cuda_res_, stream));

	//glActiveTexture(GL_TEXTURE0);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, glbuf_);
	// Select the appropriate texture
	glBindTexture(GL_TEXTURE_2D, glid_);

	glPixelStorei(GL_UNPACK_ROW_LENGTH, stride_);
	// Make a texture from the buffer
	if (type_ == Type::BGRA) {
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
	} else {
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_RED, GL_FLOAT, NULL);
	}
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

	mtx_.unlock();
}

unsigned int GLTexture::texture() const {
	if (glbuf_ < std::numeric_limits<unsigned int>::max()) {
		/*//glActiveTexture(GL_TEXTURE0);
		glBindBuffer( GL_PIXEL_UNPACK_BUFFER, glbuf_);
		// Select the appropriate texture
		glBindTexture( GL_TEXTURE_2D, glid_);
		// Make a texture from the buffer
		glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
		glBindBuffer( GL_PIXEL_UNPACK_BUFFER, 0);*/

		return glid_;
	} else {
		throw FTL_Error("No OpenGL texture; use make() first");
	}
}

void GLTexture::copyFrom(const ftl::cuda::TextureObject<uchar4> &buffer, cudaStream_t stream) {
	if (buffer.width() == 0 || buffer.height() == 0) {
		return;
	}

	make(buffer.width(), buffer.height(), ftl::utility::GLTexture::Type::BGRA);
	auto dst = map(stream);
	cudaSafeCall(cudaMemcpy2D(	dst.data, dst.step, buffer.devicePtr(), buffer.pitch(),
								buffer.width()*4, buffer.height(), cudaMemcpyDeviceToDevice));
	unmap(stream);
}

void GLTexture::copyFrom(const cv::Mat &im, cudaStream_t stream) {

	if (im.rows == 0 || im.cols == 0 || im.channels() != 4 || im.type() != CV_8UC4) {
		LOG(ERROR) << __FILE__ << ":" << __LINE__ << ": " << "bad OpenCV format";
		return;
	}

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	make(im.cols, im.rows, ftl::utility::GLTexture::Type::BGRA);

	auto dst = map(stream);
	dst.upload(im);
	unmap(stream);
}

void GLTexture::copyFrom(const cv::cuda::GpuMat &im, cudaStream_t stream) {

	if (im.rows == 0 || im.cols == 0 || im.channels() != 4 || im.type() != CV_8UC4) {
		LOG(ERROR) << __FILE__ << ":" << __LINE__ << ": " << "bad OpenCV format";
		return;
	}

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	make(im.cols, im.rows, ftl::utility::GLTexture::Type::BGRA);
	auto dst = map(stream);
	im.copyTo(dst, cvstream);
	unmap(stream);
}
