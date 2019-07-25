#include <ftl/cuda_common.hpp>

using ftl::cuda::TextureObject;

template <>
void TextureObject<uchar4>::upload(const cv::Mat &m, cudaStream_t stream) {
    cudaSafeCall(cudaMemcpy2DAsync(devicePtr(), pitch(), m.data, m.step, m.cols * sizeof(uchar4), m.rows, cudaMemcpyHostToDevice, stream));
}

template <>
void TextureObject<float>::upload(const cv::Mat &m, cudaStream_t stream) {
    cudaSafeCall(cudaMemcpy2DAsync(devicePtr(), pitch(), m.data, m.step, m.cols * sizeof(float), m.rows, cudaMemcpyHostToDevice, stream));
}

template <>
void TextureObject<float2>::upload(const cv::Mat &m, cudaStream_t stream) {
    cudaSafeCall(cudaMemcpy2DAsync(devicePtr(), pitch(), m.data, m.step, m.cols * sizeof(float2), m.rows, cudaMemcpyHostToDevice, stream));
}

template <>
void TextureObject<float4>::upload(const cv::Mat &m, cudaStream_t stream) {
    cudaSafeCall(cudaMemcpy2DAsync(devicePtr(), pitch(), m.data, m.step, m.cols * sizeof(float4), m.rows, cudaMemcpyHostToDevice, stream));
}

template <>
void TextureObject<uchar>::upload(const cv::Mat &m, cudaStream_t stream) {
    cudaSafeCall(cudaMemcpy2DAsync(devicePtr(), pitch(), m.data, m.step, m.cols * sizeof(uchar), m.rows, cudaMemcpyHostToDevice, stream));
}


template <>
void TextureObject<uchar4>::download(cv::Mat &m, cudaStream_t stream) const {
	m.create(height(), width(), CV_8UC4);
	cudaSafeCall(cudaMemcpy2DAsync(m.data, m.step, devicePtr(), pitch(), m.cols * sizeof(uchar4), m.rows, cudaMemcpyDeviceToHost, stream));
}

template <>
void TextureObject<float>::download(cv::Mat &m, cudaStream_t stream) const {
	m.create(height(), width(), CV_32FC1);
	cudaSafeCall(cudaMemcpy2DAsync(m.data, m.step, devicePtr(), pitch(), m.cols * sizeof(float), m.rows, cudaMemcpyDeviceToHost, stream));
}

template <>
void TextureObject<float2>::download(cv::Mat &m, cudaStream_t stream) const {
	m.create(height(), width(), CV_32FC2);
	cudaSafeCall(cudaMemcpy2DAsync(m.data, m.step, devicePtr(), pitch(), m.cols * sizeof(float2), m.rows, cudaMemcpyDeviceToHost, stream));
}

template <>
void TextureObject<float4>::download(cv::Mat &m, cudaStream_t stream) const {
	m.create(height(), width(), CV_32FC4);
	cudaSafeCall(cudaMemcpy2DAsync(m.data, m.step, devicePtr(), pitch(), m.cols * sizeof(float4), m.rows, cudaMemcpyDeviceToHost, stream));
}

template <>
void TextureObject<uchar>::download(cv::Mat &m, cudaStream_t stream) const {
	m.create(height(), width(), CV_8UC1);
	cudaSafeCall(cudaMemcpy2DAsync(m.data, m.step, devicePtr(), pitch(), m.cols * sizeof(uchar), m.rows, cudaMemcpyDeviceToHost, stream));
}
