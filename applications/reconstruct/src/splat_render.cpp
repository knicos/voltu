#include "splat_render.hpp"
#include "splat_render_cuda.hpp"
#include "compactors.hpp"
#include "depth_camera_cuda.hpp"

using ftl::render::Splatter;

Splatter::Splatter(ftl::voxhash::SceneRep *scene) : scene_(scene) {

}

Splatter::~Splatter() {

}

void Splatter::render(ftl::rgbd::Source *src, cudaStream_t stream) {
	if (!src->isReady()) return;

	const auto &camera = src->parameters();

	cudaSafeCall(cudaSetDevice(scene_->getCUDADevice()));

	// Create buffers if they don't exists
	if ((unsigned int)depth1_.width() != camera.width || (unsigned int)depth1_.height() != camera.height) {
		depth1_ = ftl::cuda::TextureObject<int>(camera.width, camera.height);
	}
	if ((unsigned int)colour1_.width() != camera.width || (unsigned int)colour1_.height() != camera.height) {
		colour1_ = ftl::cuda::TextureObject<uchar4>(camera.width, camera.height);
	}
	if ((unsigned int)depth2_.width() != camera.width || (unsigned int)depth2_.height() != camera.height) {
		depth2_ = ftl::cuda::TextureObject<float>(camera.width, camera.height);
	}
	if ((unsigned int)colour2_.width() != camera.width || (unsigned int)colour2_.height() != camera.height) {
		colour2_ = ftl::cuda::TextureObject<uchar4>(camera.width, camera.height);
	}

	// Parameters object to pass to CUDA describing the camera
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

	//ftl::cuda::compactifyAllocated(scene_->getHashData(), scene_->getHashParams(), stream);
	//LOG(INFO) << "Occupied: " << scene_->getOccupiedCount();

	if (scene_->value("voxels", false)) {
		// TODO:(Nick) Stereo for voxel version
		ftl::cuda::isosurface_point_image(scene_->getHashData(), depth1_, params, stream);
		ftl::cuda::splat_points(depth1_, depth2_, params, stream);
		ftl::cuda::dibr(depth2_, colour1_, scene_->cameraCount(), params, stream);
		src->writeFrames(colour1_, depth2_, stream);
	} else {
		//ftl::cuda::clear_colour(colour1_, stream);
		ftl::cuda::clear_depth(depth1_, stream);
		ftl::cuda::clear_depth(depth2_, stream);
		ftl::cuda::dibr(depth1_, colour1_, scene_->cameraCount(), params, stream);
		//ftl::cuda::hole_fill(depth1_, depth2_, params.camera, stream);

		// Splat the depth from first DIBR to expand the points
		ftl::cuda::splat_points(depth1_, depth2_, params, stream);

		// Alternative to above...
		//ftl::cuda::mls_resample(depth1_, colour1_, depth2_, scene_->getHashParams(), scene_->cameraCount(), params, stream);


		// Use reverse sampling to obtain more colour details
		// Should use nearest neighbor point depths to verify which camera to use
		//ftl::cuda::dibr(depth2_, colour1_, scene_->cameraCount(), params, stream);

		if (src->getChannel() == ftl::rgbd::kChanDepth) {
			ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
			src->writeFrames(colour1_, depth2_, stream);
		} else if (src->getChannel() == ftl::rgbd::kChanRight) {
			// Adjust pose to right eye position
			Eigen::Affine3f transform(Eigen::Translation3f(camera.baseline,0.0f,0.0f));
			Eigen::Matrix4f tmat = transform.matrix();
			Eigen::Matrix4f rmat = src->getPose().cast<float>();
			rmat(3,0) = 0.0f;
			rmat(3,1) = 0.0f;
			rmat(3,2) = 0.0f;
			Eigen::Matrix4f matrix =  (rmat * transform.matrix()) + src->getPose().cast<float>();
			params.m_viewMatrix = MatrixConversion::toCUDA(matrix.inverse());
			params.m_viewMatrixInverse = MatrixConversion::toCUDA(matrix);

			ftl::cuda::clear_depth(depth1_, stream);
			ftl::cuda::dibr(depth1_, colour2_, scene_->cameraCount(), params, stream);
			src->writeFrames(colour1_, colour2_, stream);
		} else {
			src->writeFrames(colour1_, depth2_, stream);
		}
	}

	//ftl::cuda::median_filter(depth1_, depth2_, stream);
	//ftl::cuda::splat_points(depth1_, depth2_, params, stream);

	// TODO: Second pass
}

void Splatter::setOutputDevice(int device) {
	device_ = device;
}
