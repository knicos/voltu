#ifndef _FTL_GUI_GLTEXTURE_HPP_
#define _FTL_GUI_GLTEXTURE_HPP_

#include <opencv2/opencv.hpp>

namespace ftl {
namespace gui {

class GLTexture {
	public:
	GLTexture();
	~GLTexture();

	void update(cv::Mat &m);
	unsigned int texture() const { return glid_; }

	private:
	unsigned int glid_;
};

}
}

#endif  // _FTL_GUI_GLTEXTURE_HPP_
