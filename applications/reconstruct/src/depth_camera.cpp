#include <ftl/depth_camera.hpp>
#include "depth_camera_cuda.hpp"
#include <opencv2/core/cuda_stream_accessor.hpp>

using ftl::voxhash::DepthCamera;
using ftl::voxhash::DepthCameraCUDA;

DepthCamera::DepthCamera() {
	depth_tex_ = nullptr;
	depth2_tex_ = nullptr;
	points_tex_ = nullptr;
	colour_tex_ = nullptr;
	normal_tex_ = nullptr;
}

void DepthCamera::alloc(const DepthCameraParams& params, bool withNormals) { //! todo resizing???
	depth_tex_ = new ftl::cuda::TextureObject<float>(params.m_imageWidth, params.m_imageHeight);
	depth2_tex_ = new ftl::cuda::TextureObject<int>(params.m_imageWidth, params.m_imageHeight);
	points_tex_ = new ftl::cuda::TextureObject<float4>(params.m_imageWidth, params.m_imageHeight);
	colour_tex_ = new ftl::cuda::TextureObject<uchar4>(params.m_imageWidth, params.m_imageHeight);
	data.depth = depth_tex_->cudaTexture();
	data.depth2 = depth2_tex_->cudaTexture();
	data.points = points_tex_->cudaTexture();
	data.colour = colour_tex_->cudaTexture();
	data.params = params;

	if (withNormals) {
		normal_tex_ = new ftl::cuda::TextureObject<float4>(params.m_imageWidth, params.m_imageHeight);
		data.normal = normal_tex_->cudaTexture();
	} else {
		data.normal = 0;
	}
}

void DepthCamera::free() {
	delete depth_tex_;
	delete colour_tex_;
	delete depth2_tex_;
	delete points_tex_;
	if (normal_tex_) delete normal_tex_;
}

void DepthCamera::updateData(const cv::Mat &depth, const cv::Mat &rgb, cv::cuda::Stream &stream) {
	depth_tex_->upload(depth, cv::cuda::StreamAccessor::getStream(stream));
	colour_tex_->upload(rgb, cv::cuda::StreamAccessor::getStream(stream));
	//if (normal_mat_) {
	//	_computeNormals(cv::cuda::StreamAccessor::getStream(stream));
	//}
}

void DepthCamera::_computeNormals(cudaStream_t stream) {
	//ftl::cuda::point_cloud((float3*)point_mat_->data, data, stream);
	//ftl::cuda::compute_normals((float3*)point_mat_->data, normal_tex_, stream);
}
