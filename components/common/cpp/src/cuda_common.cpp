#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <ftl/cuda_common.hpp>

using ftl::cuda::TextureObjectBase;
using ftl::cuda::BufferBase;

static int dev_to_use = 0;
static int dev_count = 0;
static std::vector<cudaDeviceProp> properties;

bool ftl::cuda::initialise() {
	// Do an initial CUDA check
	cudaSafeCall(cudaGetDeviceCount(&dev_count));
	CHECK_GE(dev_count, 1) << "No CUDA devices found";

	LOG(INFO) << "CUDA Devices (" << dev_count << "):";

	properties.resize(dev_count);
	for (int i=0; i<dev_count; i++) {
		cudaSafeCall(cudaGetDeviceProperties(&properties[i], i));
		LOG(INFO) << " - " << properties[i].name;
	}

	return true;
}

bool ftl::cuda::hasCompute(int major, int minor) {
	int dev = -1;
	cudaSafeCall(cudaGetDevice(&dev));

	if (dev > 0) {
		return properties[dev].major > major ||
			(properties[dev].major == major && properties[dev].minor >= minor);
	}
	return false;
}

int ftl::cuda::deviceCount() {
	return dev_count;
}

int ftl::cuda::getDevice() {
	return dev_to_use;
}

void ftl::cuda::setDevice(int id) {
	dev_to_use = id;
	ftl::cuda::setDevice();
}

void ftl::cuda::setDevice() {
	cudaSafeCall(cudaSetDevice(dev_to_use));
}

TextureObjectBase::~TextureObjectBase() {
	free();
}

TextureObjectBase::TextureObjectBase(TextureObjectBase &&o) {
	needsfree_ = o.needsfree_;
	needsdestroy_ = o.needsdestroy_;
	ptr_ = o.ptr_;
	texobj_ = o.texobj_;
	cvType_ = o.cvType_;
	width_ = o.width_;
	height_ = o.height_;
	pitch_ = o.pitch_;
	pitch2_ = o.pitch2_;

	o.ptr_ = nullptr;
	o.needsfree_ = false;
	o.texobj_ = 0;
	o.needsdestroy_ = false;
}

TextureObjectBase &TextureObjectBase::operator=(TextureObjectBase &&o) {
	free();

	needsfree_ = o.needsfree_;
	needsdestroy_ = o.needsdestroy_;
	ptr_ = o.ptr_;
	texobj_ = o.texobj_;
	cvType_ = o.cvType_;
	width_ = o.width_;
	height_ = o.height_;
	pitch_ = o.pitch_;
	pitch2_ = o.pitch2_;

	o.ptr_ = nullptr;
	o.needsfree_ = false;
	o.texobj_ = 0;
	o.needsdestroy_ = false;
	return *this;
}

void TextureObjectBase::free() {
	if (needsfree_) {
		if (texobj_ != 0) cudaSafeCall( cudaDestroyTextureObject (texobj_) );
		if (ptr_) cudaFree(ptr_);
		ptr_ = nullptr;
		texobj_ = 0;
		cvType_ = -1;
	} else if (needsdestroy_) {
		if (texobj_ != 0) cudaSafeCall( cudaDestroyTextureObject (texobj_) );
		texobj_ = 0;
		cvType_ = -1;
	}
}

void TextureObjectBase::upload(const cv::Mat &m, cudaStream_t stream) {
    cudaSafeCall(cudaMemcpy2DAsync(devicePtr(), pitch(), m.data, m.step, m.cols * m.elemSize(), m.rows, cudaMemcpyHostToDevice, stream));
}

void TextureObjectBase::download(cv::Mat &m, cudaStream_t stream) const {
	m.create(height(), width(), cvType_);
	cudaSafeCall(cudaMemcpy2DAsync(m.data, m.step, devicePtr(), pitch(), m.cols * m.elemSize(), m.rows, cudaMemcpyDeviceToHost, stream));
}



BufferBase::~BufferBase() {
	free();
}

BufferBase::BufferBase(BufferBase &&o) {
	needsfree_ = o.needsfree_;
	ptr_ = o.ptr_;
	cvType_ = o.cvType_;
	width_ = o.width_;
	height_ = o.height_;
	pitch_ = o.pitch_;
	pitch2_ = o.pitch2_;

	o.ptr_ = nullptr;
	o.needsfree_ = false;
}

BufferBase &BufferBase::operator=(BufferBase &&o) {
	free();

	needsfree_ = o.needsfree_;
	ptr_ = o.ptr_;
	cvType_ = o.cvType_;
	width_ = o.width_;
	height_ = o.height_;
	pitch_ = o.pitch_;
	pitch2_ = o.pitch2_;

	o.ptr_ = nullptr;
	o.needsfree_ = false;
	return *this;
}

void BufferBase::free() {
	if (needsfree_) {
		if (ptr_) cudaFree(ptr_);
		ptr_ = nullptr;
		cvType_ = -1;
	}
}

void BufferBase::upload(const cv::Mat &m, cudaStream_t stream) {
    cudaSafeCall(cudaMemcpy2DAsync(devicePtr(), pitch(), m.data, m.step, m.cols * m.elemSize(), m.rows, cudaMemcpyHostToDevice, stream));
}

void BufferBase::download(cv::Mat &m, cudaStream_t stream) const {
	m.create(height(), width(), cvType_);
	cudaSafeCall(cudaMemcpy2DAsync(m.data, m.step, devicePtr(), pitch(), m.cols * m.elemSize(), m.rows, cudaMemcpyDeviceToHost, stream));
}