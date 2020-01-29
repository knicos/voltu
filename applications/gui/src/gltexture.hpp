#ifndef _FTL_GUI_GLTEXTURE_HPP_
#define _FTL_GUI_GLTEXTURE_HPP_

#include <opencv2/core/mat.hpp>

namespace ftl {
namespace gui {

class GLTexture {
	public:
	GLTexture();
	~GLTexture();

	void update(cv::Mat &m);
	unsigned int texture() const { return glid_; }
	bool isValid() const { return glid_ != std::numeric_limits<unsigned int>::max(); }

	private:
	unsigned int glid_;
};

}
}

#endif  // _FTL_GUI_GLTEXTURE_HPP_
