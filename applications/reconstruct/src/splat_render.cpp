#include "splat_render.hpp"
#include "splat_render_cuda.hpp"
#include "compactors.hpp"

using ftl::render::Splatter;

Splatter::Splatter(ftl::voxhash::SceneRep *scene) : scene_(scene) {

}

Splatter::~Splatter() {

}

void Splatter::render(ftl::rgbd::Source *src, cudaStream_t stream) {
	if (!src->isReady()) return;

	const auto &camera = src->parameters();

	cudaSafeCall(cudaSetDevice(scene_->getCUDADevice()));

	if (depth1_.width() != camera.width || depth1_.height() != camera.height) {
		depth1_ = ftl::cuda::TextureObject<uint>(camera.width, camera.height);
	}
	if (colour1_.width() != camera.width || colour1_.height() != camera.height) {
		colour1_ = ftl::cuda::TextureObject<uchar4>(camera.width, camera.height);
	}
	if (depth2_.width() != camera.width || depth2_.height() != camera.height) {
		depth2_ = ftl::cuda::TextureObject<float>(camera.width, camera.height);
	}
	if (colour2_.width() != camera.width || colour2_.height() != camera.height) {
		colour2_ = ftl::cuda::TextureObject<uchar4>(camera.width, camera.height);
	}

	SplatParams params;
	params.m_flags = 0;
	params.m_viewMatrix = MatrixConversion::toCUDA(src->getPose().cast<float>().inverse());
	params.m_viewMatrixInverse = MatrixConversion::toCUDA(src->getPose().cast<float>());
	params.voxelSize = scene_->getHashParams().m_virtualVoxelSize;
	params.camera.flags = 0;
	params.camera.fx = camera.fx;
	params.camera.fy = camera.fy;
	params.camera.mx = -camera.cx;
	params.camera.my = -camera.cy;
	params.camera.m_imageWidth = camera.width;
	params.camera.m_imageHeight = camera.height;
	params.camera.m_sensorDepthWorldMax = camera.maxDepth;
	params.camera.m_sensorDepthWorldMin = camera.minDepth;

	ftl::cuda::compactifyVisible(scene_->getHashData(), scene_->getHashParams(), params.camera, stream);
	ftl::cuda::isosurface_point_image(scene_->getHashData(), depth1_, colour1_, params, stream);
	ftl::cuda::splat_points(depth1_, colour1_, depth2_, colour2_, params, stream);

	// TODO: Second pass

	src->writeFrames(colour2_, depth2_, stream);
}

void Splatter::setOutputDevice(int device) {
	device_ = device;
}
