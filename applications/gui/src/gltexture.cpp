#include "gltexture.hpp"

#include <nanogui/opengl.h>
#include <loguru.hpp>

using ftl::gui::GLTexture;

GLTexture::GLTexture() {
	glid_ = std::numeric_limits<unsigned int>::max();
}

GLTexture::~GLTexture() {
	//glDeleteTextures(1, &glid_);
}

void GLTexture::update(cv::Mat &m) {
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
		glBindTexture(GL_TEXTURE_2D, glid_);
		// TODO Allow for other formats
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m.cols, m.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, m.data);
	}
	auto err = glGetError();
	if (err != 0) LOG(ERROR) << "OpenGL Texture error: " << err;
}
