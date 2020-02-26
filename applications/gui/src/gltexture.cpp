#include "gltexture.hpp"

#include <nanogui/opengl.h>
#include <loguru.hpp>

#include <ftl/cuda_common.hpp>
#include <cuda_gl_interop.h>

#include <ftl/exception.hpp>

using ftl::gui::GLTexture;

GLTexture::GLTexture() {
	glid_ = std::numeric_limits<unsigned int>::max();
	glbuf_ = std::numeric_limits<unsigned int>::max();
	cuda_res_ = nullptr;
	width_ = 0;
	height_ = 0;
	changed_ = true;
}

GLTexture::~GLTexture() {
	//glDeleteTextures(1, &glid_);
}

void GLTexture::update(cv::Mat &m) {
	LOG(INFO) << "DEPRECATED";
	if (m.rows == 0) return;
	if (glid_ == std::numeric_limits<unsigned int>::max()) {
		glGenTextures(1, &glid_);
		glBindTexture(GL_TEXTURE_2D, glid_);
		//cv::Mat m(cv::Size(100,100), CV_8UC3);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m.cols, m.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, m.data);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	} else {
		//glBindTexture(GL_TEXTURE_2D, glid_);
		// TODO Allow for other formats
		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m.cols, m.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, m.data);
	}
	auto err = glGetError();
	if (err != 0) LOG(ERROR) << "OpenGL Texture error: " << err;
}

void GLTexture::make(int width, int height) {
	if (width != width_ || height != height_) {
		free();
	}

	width_ = width;
	height_ = height;

	if (width == 0 || height == 0) {
		throw FTL_Error("Invalid texture size");
	}

	if (glid_ == std::numeric_limits<unsigned int>::max()) {
		glGenTextures(1, &glid_);
		glBindTexture(GL_TEXTURE_2D, glid_);
		//cv::Mat m(cv::Size(100,100), CV_8UC3);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_BGRA, GL_UNSIGNED_BYTE, nullptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		//auto err = glGetError();
		//if (err != 0) LOG(ERROR) << "OpenGL Texture error: " << err;

		glBindTexture(GL_TEXTURE_2D, 0);

		glGenBuffers(1, &glbuf_);
		// Make this the current UNPACK buffer (OpenGL is state-based)
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, glbuf_);
		// Allocate data for the buffer. 4-channel 8-bit image
		glBufferData(GL_PIXEL_UNPACK_BUFFER, width * height * 4, NULL, GL_DYNAMIC_COPY);

		cudaSafeCall(cudaGraphicsGLRegisterBuffer(&cuda_res_, glbuf_, cudaGraphicsRegisterFlagsWriteDiscard));
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
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
	void *devptr;
	size_t size;
	cudaSafeCall(cudaGraphicsMapResources(1, &cuda_res_, stream));
	cudaSafeCall(cudaGraphicsResourceGetMappedPointer(&devptr, &size, cuda_res_));
	return cv::cuda::GpuMat(height_, width_, CV_8UC4, devptr);
}

void GLTexture::unmap(cudaStream_t stream) {
	cudaSafeCall(cudaGraphicsUnmapResources(1, &cuda_res_, stream));
	changed_ = true;

	//glActiveTexture(GL_TEXTURE0);
	glBindBuffer( GL_PIXEL_UNPACK_BUFFER, glbuf_);
	// Select the appropriate texture
	glBindTexture( GL_TEXTURE_2D, glid_);
	// Make a texture from the buffer
	glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
	glBindBuffer( GL_PIXEL_UNPACK_BUFFER, 0);
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
		return glid_;
	}
}
