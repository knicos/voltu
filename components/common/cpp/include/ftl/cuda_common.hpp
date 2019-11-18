#ifndef _FTL_CUDA_COMMON_HPP_
#define _FTL_CUDA_COMMON_HPP_

#include <ftl/config.h>
#include <ftl/traits.hpp>

#if defined HAVE_CUDA

#include <ftl/cuda_util.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda/common.hpp>

#ifndef __CUDACC__
#include <loguru.hpp>
#include <exception>
#endif

/* Grid stride loop macros */
#define STRIDE_Y(I,N) int I = blockIdx.y * blockDim.y + threadIdx.y; I < N; I += blockDim.y * gridDim.y
#define STRIDE_X(I,N) int I = blockIdx.x * blockDim.x + threadIdx.x; I < N; I += blockDim.x * gridDim.x

namespace ftl {
namespace cuda {

bool initialise();

bool hasCompute(int major, int minor);

int deviceCount();

template <typename T>
struct Float;

template <> struct Float<float> { typedef float type; };
template <> struct Float<int> { typedef float type; };
template <> struct Float<float4> { typedef float4 type; };
template <> struct Float<uchar4> { typedef float4 type; };
template <> struct Float<short2> { typedef float2 type; };

template <typename T>
struct ScaleValue;

template <> struct ScaleValue<uchar4> { static constexpr float value = 255.0f; };
template <> struct ScaleValue<float> { static constexpr float value = 1.0f; };
template <> struct ScaleValue<float4> { static constexpr float value = 1.0f; };

/**
 * Represent a CUDA texture object. Instances of this class can be used on both
 * host and device. A texture object base cannot be constructed directly, it
 * must be constructed via a template TextureObject class.
 */
class TextureObjectBase {
	public:
	__host__ __device__ TextureObjectBase()
			: texobj_(0), pitch_(0), pitch2_(0), width_(0), height_(0),
			  ptr_(nullptr), needsfree_(false), needsdestroy_(false),
			  cvType_(-1) {};
	~TextureObjectBase();

	// Remove ability to copy object directly, instead must use
	// templated derivative TextureObject.
	TextureObjectBase(const TextureObjectBase &)=delete;
	TextureObjectBase &operator=(const TextureObjectBase &)=delete;

	TextureObjectBase(TextureObjectBase &&);
	TextureObjectBase &operator=(TextureObjectBase &&);

	inline size_t pitch() const { return pitch_; }
	inline size_t pixelPitch() const { return pitch2_; }
	inline uchar *devicePtr() const { return ptr_; };
	__host__ __device__ inline uchar *devicePtr(int v) const { return &ptr_[v*pitch_]; }
	__host__ __device__ inline int width() const { return width_; }
	__host__ __device__ inline int height() const { return height_; }
	__host__ __device__ inline cudaTextureObject_t cudaTexture() const { return texobj_; }

	void upload(const cv::Mat &, cudaStream_t stream=0);
	void download(cv::Mat &, cudaStream_t stream=0) const;
	
	__host__ void free();

	inline int cvType() const { return cvType_; }
	
	protected:
	cudaTextureObject_t texobj_;
	size_t pitch_;
	size_t pitch2_; 		// in T units
	int width_;
	int height_;
	uchar *ptr_;			// Device memory pointer
	bool needsfree_;		// We manage memory, so free it
	bool needsdestroy_;		// The texture object needs to be destroyed
	int cvType_;				// Used to validate casting
};

/**
 * Create and manage CUDA texture objects with a particular pixel data type.
 * Note: it is not possible to create texture objects for certain types,
 * specificially for 3 channel types.
 */
template <typename T>
class TextureObject : public TextureObjectBase {
	public:
	typedef T type;

	static_assert((16u % sizeof(T)) == 0, "Channel format must be aligned with 16 bytes");

	__host__ __device__ TextureObject() : TextureObjectBase() {};
	explicit TextureObject(const cv::cuda::GpuMat &d, bool interpolated=false);
	explicit TextureObject(const cv::cuda::PtrStepSz<T> &d);
	TextureObject(T *ptr, int pitch, int width, int height);
	TextureObject(size_t width, size_t height);
	TextureObject(const TextureObject<T> &t);
	__host__ __device__ TextureObject(TextureObject<T> &&);
	~TextureObject();

	TextureObject<T> &operator=(const TextureObject<T> &);
	__host__ __device__ TextureObject<T> &operator=(TextureObject<T> &&);

	operator cv::cuda::GpuMat();

	void create(const cv::Size &);
	void create(int w, int h);

	__host__ __device__ T *devicePtr() const { return (T*)(ptr_); };
	__host__ __device__ T *devicePtr(int v) const { return &(T*)(ptr_)[v*pitch2_]; }

	#ifdef __CUDACC__
	__device__ inline T tex2D(int u, int v) const { return ::tex2D<T>(texobj_, u, v); }
	__device__ inline T tex2D(unsigned int u, unsigned int v) const { return ::tex2D<T>(texobj_, (int)u, (int)v); }
	__device__ inline typename Float<T>::type tex2D(float u, float v) const { return ::tex2D<typename Float<T>::type>(texobj_, u, v) * ScaleValue<T>::value; }
	#endif

	__host__ __device__ inline const T &operator()(int u, int v) const { return reinterpret_cast<T*>(ptr_)[u+v*pitch2_]; }
	__host__ __device__ inline T &operator()(int u, int v) { return reinterpret_cast<T*>(ptr_)[u+v*pitch2_]; }

	/**
	 * Cast a base texture object to this type of texture object. If the
	 * underlying pixel types do not match then a bad_cast exception is thrown.
	 */
	static TextureObject<T> &cast(TextureObjectBase &);
};

#ifndef __CUDACC__
template <typename T>
TextureObject<T> &TextureObject<T>::cast(TextureObjectBase &b) {
	if (b.cvType() != ftl::traits::OpenCVType<T>::value) {
		LOG(ERROR) << "Bad cast of texture object";
		throw std::bad_cast();
	}
	return reinterpret_cast<TextureObject<T>&>(b);
}

/**
 * Create a 2D array texture from an OpenCV GpuMat object.
 */
template <typename T>
TextureObject<T>::TextureObject(const cv::cuda::GpuMat &d, bool interpolated) {
	// GpuMat must have correct data type
	CHECK(d.type() == ftl::traits::OpenCVType<T>::value);

	cudaResourceDesc resDesc;
	memset(&resDesc, 0, sizeof(resDesc));
	resDesc.resType = cudaResourceTypePitch2D;
	resDesc.res.pitch2D.devPtr = d.data;
	resDesc.res.pitch2D.pitchInBytes = d.step;
	resDesc.res.pitch2D.desc = cudaCreateChannelDesc<T>();
	resDesc.res.pitch2D.width = d.cols;
	resDesc.res.pitch2D.height = d.rows;

	cudaTextureDesc texDesc;
	// cppcheck-suppress memsetClassFloat
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode = (interpolated) ? cudaReadModeNormalizedFloat : cudaReadModeElementType;
	if (interpolated) texDesc.filterMode = cudaFilterModeLinear;

	cudaTextureObject_t tex = 0;
	cudaSafeCall(cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL));
	texobj_ = tex;
	pitch_ = d.step;
	pitch2_ = pitch_ / sizeof(T);
	ptr_ = d.data;
	width_ = d.cols;
	height_ = d.rows;
	needsfree_ = false;
	cvType_ = ftl::traits::OpenCVType<T>::value;
	needsdestroy_ = true;
}

#endif  // __CUDACC__

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
	// cppcheck-suppress memsetClassFloat
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode = cudaReadModeElementType;
	//if (std::is_same<T,uchar4>::value) texDesc.filterMode = cudaFilterModeLinear;

	cudaTextureObject_t tex = 0;
	cudaSafeCall(cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL));
	texobj_ = tex;
	pitch_ = d.step;
	pitch2_ = pitch_ / sizeof(T);
	ptr_ = d.data;
	width_ = d.cols;
	height_ = d.rows;
	needsfree_ = false;
	cvType_ = ftl::traits::OpenCVType<T>::value;
	needsdestroy_ = true;
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
	// cppcheck-suppress memsetClassFloat
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.readMode = cudaReadModeElementType;
	//if (std::is_same<T,uchar4>::value) texDesc.filterMode = cudaFilterModeLinear;

	cudaTextureObject_t tex = 0;
	cudaSafeCall(cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL));
	texobj_ = tex;
	pitch_ = pitch;
	pitch2_ = pitch_ / sizeof(T);
	ptr_ = ptr;
	width_ = width;
	height_ = height;
	needsfree_ = false;
	cvType_ = ftl::traits::OpenCVType<T>::value;
	needsdestroy_ = true;
}

template <typename T>
TextureObject<T>::TextureObject(size_t width, size_t height) {
	cudaMallocPitch((void**)&ptr_,&pitch_,width*sizeof(T),height);
	cudaTextureObject_t tex = 0;

	// Must be an even
	//if (!(sizeof(T) & 0x1)) {
		cudaResourceDesc resDesc;
		memset(&resDesc, 0, sizeof(resDesc));
		resDesc.resType = cudaResourceTypePitch2D;
		resDesc.res.pitch2D.devPtr = ptr_;
		resDesc.res.pitch2D.pitchInBytes = pitch_;
		resDesc.res.pitch2D.desc = cudaCreateChannelDesc<T>();
		resDesc.res.pitch2D.width = width;
		resDesc.res.pitch2D.height = height;

		cudaTextureDesc texDesc;
		// cppcheck-suppress memsetClassFloat
		memset(&texDesc, 0, sizeof(texDesc));
		texDesc.readMode = cudaReadModeElementType;
		//if (std::is_same<T,uchar4>::value) texDesc.filterMode = cudaFilterModeLinear;
		cudaSafeCall(cudaCreateTextureObject(&tex, &resDesc, &texDesc, NULL));
	//}

	texobj_ = tex;
	width_ = (int)width;
	height_ = (int)height;
	needsfree_ = true;
	pitch2_ = pitch_ / sizeof(T);
	cvType_ = ftl::traits::OpenCVType<T>::value;
	needsdestroy_ = true;
}

#ifndef __CUDACC__
template <typename T>
void TextureObject<T>::create(const cv::Size &s) {
	create(s.width, s.height);
}

template <typename T>
void TextureObject<T>::create(int w, int h) {
	if (width_ != w || height_ != h) {
		*this = std::move(TextureObject<T>(w, h));
	}
}
#endif

template <typename T>
TextureObject<T>::TextureObject(const TextureObject<T> &p) {
	texobj_ = p.texobj_;
	ptr_ = p.ptr_;
	width_ = p.width_;
	height_ = p.height_;
	pitch_ = p.pitch_;
	pitch2_ = pitch_ / sizeof(T);
	cvType_ = ftl::traits::OpenCVType<T>::value;
	needsfree_ = false;
	needsdestroy_ = false;
}

template <typename T>
TextureObject<T>::TextureObject(TextureObject<T> &&p) {
	texobj_ = p.texobj_;
	ptr_ = p.ptr_;
	width_ = p.width_;
	height_ = p.height_;
	pitch_ = p.pitch_;
	pitch2_ = pitch_ / sizeof(T);
	needsfree_ = p.needsfree_;
	needsdestroy_ = p.needsdestroy_;
	p.texobj_ = 0;
	p.needsfree_ = false;
	p.needsdestroy_ = false;
	p.ptr_ = nullptr;
	cvType_ = ftl::traits::OpenCVType<T>::value;
}

template <typename T>
TextureObject<T> &TextureObject<T>::operator=(const TextureObject<T> &p) {
	free();
	texobj_ = p.texobj_;
	ptr_ = p.ptr_;
	width_ = p.width_;
	height_ = p.height_;
	pitch_ = p.pitch_;
	pitch2_ = pitch_ / sizeof(T);
	cvType_ = ftl::traits::OpenCVType<T>::value;
	needsfree_ = false;
	needsdestroy_ = false;
	return *this;
}

template <typename T>
TextureObject<T> &TextureObject<T>::operator=(TextureObject<T> &&p) {
	free();
	texobj_ = p.texobj_;
	ptr_ = p.ptr_;
	width_ = p.width_;
	height_ = p.height_;
	pitch_ = p.pitch_;
	pitch2_ = pitch_ / sizeof(T);
	needsfree_ = p.needsfree_;
	needsdestroy_ = p.needsdestroy_;
	p.texobj_ = 0;
	p.needsfree_ = false;
	p.needsdestroy_ = false;
	p.ptr_ = nullptr;
	cvType_ = ftl::traits::OpenCVType<T>::value;
	return *this;
}

template <typename T>
TextureObject<T>::~TextureObject() {
	//if (needsdestroy_) cudaSafeCall( cudaDestroyTextureObject (texobj_) );
	//if (needsfree_) cudaFree(ptr_);
	free();
}

/*template <>
void TextureObject<uchar4>::upload(const cv::Mat &m, cudaStream_t stream);

template <>
void TextureObject<float>::upload(const cv::Mat &m, cudaStream_t stream);

template <>
void TextureObject<float2>::upload(const cv::Mat &m, cudaStream_t stream);

template <>
void TextureObject<float4>::upload(const cv::Mat &m, cudaStream_t stream);

template <>
void TextureObject<uchar>::upload(const cv::Mat &m, cudaStream_t stream);


template <>
void TextureObject<uchar4>::download(cv::Mat &m, cudaStream_t stream) const;

template <>
void TextureObject<float>::download(cv::Mat &m, cudaStream_t stream) const;

template <>
void TextureObject<float2>::download(cv::Mat &m, cudaStream_t stream) const;

template <>
void TextureObject<float4>::download(cv::Mat &m, cudaStream_t stream) const;

template <>
void TextureObject<uchar>::download(cv::Mat &m, cudaStream_t stream) const;*/

}
}

#endif // HAVE_CUDA

#endif // FTL_CUDA_COMMON_HPP_
