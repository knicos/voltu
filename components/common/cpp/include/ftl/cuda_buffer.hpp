/**
 * @file cuda_buffer.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 * 
 * UNUSED currently
 */

#ifndef _FTL_CUDA_BUFFER_HPP_
#define _FTL_CUDA_BUFFER_HPP_

namespace ftl {
namespace cuda {

class BufferBase {
	public:
	__host__ __device__ BufferBase()
			: pitch_(0), pitch2_(0), width_(0), height_(0),
			  ptr_(nullptr), needsfree_(false),
			  cvType_(-1) {};
	~BufferBase();

	BufferBase(const BufferBase &)=delete;
	BufferBase &operator=(const BufferBase &)=delete;

	BufferBase(BufferBase &&);
	BufferBase &operator=(BufferBase &&);

	inline size_t pitch() const { return pitch_; }
	inline size_t pixelPitch() const { return pitch2_; }
	inline uchar *devicePtr() const { return ptr_; };
	__host__ __device__ inline uchar *devicePtr(int v) const { return &ptr_[v*pitch_]; }
	__host__ __device__ inline int width() const { return width_; }
	__host__ __device__ inline int height() const { return height_; }

	void upload(const cv::Mat &, cudaStream_t stream=0);
	void download(cv::Mat &, cudaStream_t stream=0) const;
	
	__host__ void free();

	inline int cvType() const { return cvType_; }
	
	protected:
	size_t pitch_;
	size_t pitch2_; 		// in T units
	int width_;
	int height_;
	uchar *ptr_;			// Device memory pointer
	bool needsfree_;		// We manage memory, so free it
	int cvType_;				// Used to validate casting
};


template <typename T>
class Buffer : public BufferBase {
	public:
	typedef T type;

	__host__ __device__ Buffer() : BufferBase() {};
	explicit Buffer(const cv::cuda::GpuMat &d);
	explicit Buffer(const cv::cuda::PtrStepSz<T> &d);
	Buffer(T *ptr, int pitch, int width, int height);
	Buffer(size_t width, size_t height);
	Buffer(const Buffer<T> &t);
	__host__ __device__ Buffer(Buffer<T> &&);
	~Buffer();

	Buffer<T> &operator=(const Buffer<T> &);
	__host__ __device__ Buffer<T> &operator=(Buffer<T> &&);

	operator cv::cuda::GpuMat();

	void create(const cv::Size &);
	void create(int w, int h);

	__host__ __device__ T *devicePtr() const { return (T*)(ptr_); };
	__host__ __device__ T *devicePtr(int v) const { return &(T*)(ptr_)[v*pitch2_]; }

	__host__ __device__ inline const T &operator()(int u, int v) const { return reinterpret_cast<T*>(ptr_)[u+v*pitch2_]; }
	__host__ __device__ inline T &operator()(int u, int v) { return reinterpret_cast<T*>(ptr_)[u+v*pitch2_]; }

	/**
	 * Cast a base texture object to this type of texture object. If the
	 * underlying pixel types do not match then a bad_cast exception is thrown.
	 */
	static Buffer<T> &cast(BufferBase &);
};

#ifndef __CUDACC__
template <typename T>
Buffer<T> &Buffer<T>::cast(BufferBase &b) {
	if (b.cvType() != ftl::traits::OpenCVType<T>::value) {
		//LOG(ERROR) << "Bad cast of texture object";
		throw std::bad_cast();
	}
	return reinterpret_cast<Buffer<T>&>(b);
}

/**
 * Create a 2D array texture from an OpenCV GpuMat object.
 */
template <typename T>
Buffer<T>::Buffer(const cv::cuda::GpuMat &d) {
	pitch_ = d.step;
	pitch2_ = pitch_ / sizeof(T);
	ptr_ = d.data;
	width_ = d.cols;
	height_ = d.rows;
	needsfree_ = false;
	cvType_ = ftl::traits::OpenCVType<T>::value;
}

#endif  // __CUDACC__

template <typename T>
Buffer<T>::Buffer(const cv::cuda::PtrStepSz<T> &d) {
	pitch_ = d.step;
	pitch2_ = pitch_ / sizeof(T);
	ptr_ = d.data;
	width_ = d.cols;
	height_ = d.rows;
	needsfree_ = false;
	cvType_ = ftl::traits::OpenCVType<T>::value;
}

template <typename T>
Buffer<T>::Buffer(T *ptr, int pitch, int width, int height) {
	pitch_ = pitch;
	pitch2_ = pitch_ / sizeof(T);
	ptr_ = ptr;
	width_ = width;
	height_ = height;
	needsfree_ = false;
	cvType_ = ftl::traits::OpenCVType<T>::value;
}

template <typename T>
Buffer<T>::Buffer(size_t width, size_t height) {
	cudaMallocPitch((void**)&ptr_,&pitch_,width*sizeof(T),height);
	width_ = (int)width;
	height_ = (int)height;
	needsfree_ = true;
	pitch2_ = pitch_ / sizeof(T);
	cvType_ = ftl::traits::OpenCVType<T>::value;
}

#ifndef __CUDACC__
template <typename T>
void Buffer<T>::create(const cv::Size &s) {
	create(s.width, s.height);
}

template <typename T>
void Buffer<T>::create(int w, int h) {
	if (width_ != w || height_ != h) {
		*this = std::move(Buffer<T>(w, h));
	}
}
#endif

template <typename T>
Buffer<T>::Buffer(const Buffer<T> &p) {
	ptr_ = p.ptr_;
	width_ = p.width_;
	height_ = p.height_;
	pitch_ = p.pitch_;
	pitch2_ = pitch_ / sizeof(T);
	cvType_ = ftl::traits::OpenCVType<T>::value;
	needsfree_ = false;
}

template <typename T>
Buffer<T>::Buffer(Buffer<T> &&p) {
	ptr_ = p.ptr_;
	width_ = p.width_;
	height_ = p.height_;
	pitch_ = p.pitch_;
	pitch2_ = pitch_ / sizeof(T);
	needsfree_ = p.needsfree_;
	p.needsfree_ = false;
	p.ptr_ = nullptr;
	cvType_ = ftl::traits::OpenCVType<T>::value;
}

template <typename T>
Buffer<T> &Buffer<T>::operator=(const Buffer<T> &p) {
	free();
	ptr_ = p.ptr_;
	width_ = p.width_;
	height_ = p.height_;
	pitch_ = p.pitch_;
	pitch2_ = pitch_ / sizeof(T);
	cvType_ = ftl::traits::OpenCVType<T>::value;
	needsfree_ = false;
	return *this;
}

template <typename T>
Buffer<T> &Buffer<T>::operator=(Buffer<T> &&p) {
	free();
	ptr_ = p.ptr_;
	width_ = p.width_;
	height_ = p.height_;
	pitch_ = p.pitch_;
	pitch2_ = pitch_ / sizeof(T);
	needsfree_ = p.needsfree_;
	p.needsfree_ = false;
	p.ptr_ = nullptr;
	cvType_ = ftl::traits::OpenCVType<T>::value;
	return *this;
}

template <typename T>
Buffer<T>::~Buffer() {
	free();
}

}
}

#endif
