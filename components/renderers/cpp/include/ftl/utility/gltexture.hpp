#pragma once

#include <opencv2/core.hpp>
#include <ftl/cuda_common.hpp>

struct cudaGraphicsResource;

namespace ftl {
namespace utility {

class GLTexture {
public:
	enum class Type {
		RGBA,
		BGRA,
		Float
	};

	explicit GLTexture();
	~GLTexture();

	bool isValid() const { return glid_ != std::numeric_limits<unsigned int>::max(); }
	int width() const { return width_; }
	int height() const { return height_; }
	Type type() const { return type_; }

	std::mutex& mutex() { return mtx_; }

	// acquire mutex before make() or free()
	void make(int width, int height, Type type);
	void free();
	unsigned int texture() const;

	cv::cuda::GpuMat map(cudaStream_t stream);
	void unmap(cudaStream_t stream);

	void copyFrom(const ftl::cuda::TextureObject<uchar4> &buf, cudaStream_t stream = cudaStreamDefault);

	void copyFrom(const cv::Mat &im, cudaStream_t stream = cudaStreamDefault);
	void copyFrom(const cv::cuda::GpuMat &im, cudaStream_t stream = cudaStreamDefault);

private:
	unsigned int glid_;
	unsigned int glbuf_;
	int width_;
	int height_;
	int stride_;

	Type type_;

	std::mutex mtx_; // for locking while in use (opengl thread calls lock() or cuda mapped)

	cudaGraphicsResource *cuda_res_;
};

}
}
