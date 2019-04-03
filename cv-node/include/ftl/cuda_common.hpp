#ifndef _FTL_CUDA_COMMON_HPP_
#define _FTL_CUDA_COMMON_HPP_

#include <ftl/config.h>

#if defined HAVE_CUDA

#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda/common.hpp>

/* Grid stride loop macros */
#define STRIDE_Y(I,N) int I = blockIdx.y * blockDim.y + threadIdx.y; I < N; I += blockDim.y * gridDim.y
#define STRIDE_X(I,N) int I = blockIdx.x * blockDim.x + threadIdx.x; I < N; I += blockDim.x * gridDim.x

namespace ftl {
namespace cuda {

/*template <typename T>
class HisteresisTexture {
	public:
	HisteresisTexture();
	~HisteresisTexture();
	
	HisteresisTexture<T> &operator=(TextureObject<T> &t);
};*/

template <typename T>
class TextureObject {
	public:
	TextureObject() : texobj_(0), ptr_(nullptr) {};
	TextureObject(const cv::cuda::PtrStepSz<T> &d);
	TextureObject(T *ptr, int pitch, int width, int height);
	TextureObject(size_t width, size_t height);
	TextureObject(const TextureObject &t);
	~TextureObject();
	
	int pitch() const { return pitch_; }
	T *devicePtr() { return ptr_; };
	__host__ __device__ T *devicePtr(int v) { return &ptr_[v*pitch2_]; }
	__host__ __device__ int width() const { return width_; }
	__host__ __device__ int height() const { return height_; }
	cudaTextureObject_t cudaTexture() const { return texobj_; }
	__device__ inline T tex2D(int u, int v) { return ::tex2D<T>(texobj_, u, v); }
	__device__ inline T tex2D(float u, float v) { return ::tex2D<T>(texobj_, u, v); }
	
	__host__ __device__ inline const T &operator()(int u, int v) const { return ptr_[u+v*pitch2_]; }
	__host__ __device__ inline T &operator()(int u, int v) { return ptr_[u+v*pitch2_]; }
	
	void free() {
		if (texobj_ != 0) cudaSafeCall( cudaDestroyTextureObject (texobj_) );
		if (ptr_ && needsfree_) cudaFree(ptr_);
		ptr_ = nullptr;
		texobj_ = 0;
	}
	
	private:
	cudaTextureObject_t texobj_;
	size_t pitch_;
	size_t pitch2_; // in T units
	int width_;
	int height_;
	T *ptr_;
	bool needsfree_;
	//bool needsdestroy_;
};

/**
 * Create a 2D array texture from an OpenCV GpuMat object.
 */
template <typename T>
TextureObject<T>::TextureObject(const cv::cuda::PtrStepSz<T> &d) {
	cudaResourceDesc resDesc;
	memset(&resDesc, 0, sizeof(resDesc));
	resDesc.resType = cudaResourceTypePitch2D;
	resDesc.res.pitch2D.devPtr = d.data;
	resDesc.res.pitch2D.pitchInBytes = d.step;
	resDesc.res.pitch2D.desc = cudaCreateChannelDesc<T>();
	resDesc.res.pitch2D.width = d.cols;
	resDesc.res.pitch2D.height = d.rows;

	cudaTextureDesc texDesc;
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode = cudaReadModeElementType;

	cudaTextureObject_t tex = 0;
	cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL);
	texobj_ = tex;
	pitch_ = d.step;
	pitch2_ = pitch_ / sizeof(T);
	ptr_ = d.data;
	width_ = d.cols;
	height_ = d.rows;
	needsfree_ = false;
	//needsdestroy_ = true;
}

/**
 * Create a 2D array texture object using a cudaMallocPitch device pointer.
 * The texture object returned must be destroyed by the caller.
 */
template <typename T>
TextureObject<T>::TextureObject(T *ptr, int pitch, int width, int height) {
	cudaResourceDesc resDesc;
	memset(&resDesc, 0, sizeof(resDesc));
	resDesc.resType = cudaResourceTypePitch2D;
	resDesc.res.pitch2D.devPtr = ptr;
	resDesc.res.pitch2D.pitchInBytes = pitch;
	resDesc.res.pitch2D.desc = cudaCreateChannelDesc<T>();
	resDesc.res.pitch2D.width = width;
	resDesc.res.pitch2D.height = height;

	cudaTextureDesc texDesc;
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode = cudaReadModeElementType;

	cudaTextureObject_t tex = 0;
	cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL);
	texobj_ = tex;
	pitch_ = pitch;
	pitch2_ = pitch_ / sizeof(T);
	ptr_ = ptr;
	width_ = width;
	height_ = height;
	needsfree_ = false;
	//needsdestroy_ = true;
}

template <typename T>
TextureObject<T>::TextureObject(size_t width, size_t height) {
	cudaMallocPitch((void**)&ptr_,&pitch_,width*sizeof(T),height);

	cudaResourceDesc resDesc;
	memset(&resDesc, 0, sizeof(resDesc));
	resDesc.resType = cudaResourceTypePitch2D;
	resDesc.res.pitch2D.devPtr = ptr_;
	resDesc.res.pitch2D.pitchInBytes = pitch_;
	resDesc.res.pitch2D.desc = cudaCreateChannelDesc<T>();
	resDesc.res.pitch2D.width = width;
	resDesc.res.pitch2D.height = height;

	cudaTextureDesc texDesc;
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode = cudaReadModeElementType;

	cudaTextureObject_t tex = 0;
	cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL);
	texobj_ = tex;
	width_ = width;
	height_ = height;
	needsfree_ = true;
	pitch2_ = pitch_ / sizeof(T);
	//needsdestroy_ = true;
}

template <typename T>
TextureObject<T>::TextureObject(const TextureObject &p) {
	texobj_ = p.texobj_;
	ptr_ = p.ptr_;
	width_ = p.width_;
	height_ = p.height_;
	pitch_ = p.pitch_;
	pitch2_ = pitch_ / sizeof(T);
	needsfree_ = p.needsfree_;
	//needsdestroy_ = false;
}

template <typename T>
TextureObject<T>::~TextureObject() {
	//if (needsdestroy_) cudaSafeCall( cudaDestroyTextureObject (texobj_) );
	//if (needsfree_) cudaFree(ptr_);
}

}
}

#endif // HAVE_CUDA

#endif // FTL_CUDA_COMMON_HPP_
