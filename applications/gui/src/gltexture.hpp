#ifndef _FTL_GUI_GLTEXTURE_HPP_
#define _FTL_GUI_GLTEXTURE_HPP_

#include <opencv2/core/mat.hpp>

#include <cuda_runtime.h>

struct cudaGraphicsResource;

namespace ftl {
namespace gui {

class GLTexture {
	public:
	enum class Type {
		RGBA,
		BGRA,
		Float
	};

	explicit GLTexture(Type);
	~GLTexture();

	void update(cv::Mat &m);
	void make(int width, int height);
	unsigned int texture() const;
	bool isValid() const { return glid_ != std::numeric_limits<unsigned int>::max(); }

	cv::cuda::GpuMat map(cudaStream_t stream);
	void unmap(cudaStream_t stream);

	void free();

	int width() const { return width_; }
	int height() const { return height_; }

	private:
	unsigned int glid_;
	unsigned int glbuf_;
	int width_;
	int height_;
	bool changed_;
	Type type_;

	cudaGraphicsResource *cuda_res_;
};

}
}

#endif  // _FTL_GUI_GLTEXTURE_HPP_
