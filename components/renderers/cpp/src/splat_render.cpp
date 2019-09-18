#include <ftl/render/splat_render.hpp>
#include "splatter_cuda.hpp"

using ftl::render::Splatter;
using ftl::rgbd::Channel;
using ftl::rgbd::Format;
using cv::cuda::GpuMat;

Splatter::Splatter(nlohmann::json &config, const ftl::rgbd::FrameSet &fs) : ftl::render::Renderer(config), scene_(fs) {

}

Splatter::~Splatter() {

}

bool Splatter::render(ftl::rgbd::VirtualSource *src, cudaStream_t stream) {
	if (!src->isReady()) return;

	const auto &camera = src->parameters();

	//cudaSafeCall(cudaSetDevice(scene_->getCUDADevice()));

	output_.create<GpuMat>(Channel::Depth, Format<float>(camera.width, camera.height));
	output_.create<GpuMat>(Channel::Colour, Format<uchar4>(camera.width, camera.height));

	temp_.create<GpuMat>(Channel::Colour, Format<float4>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Colour2, Format<uchar4>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Confidence, Format<float>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth2, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Normals, Format<float4>(camera.width, camera.height));

	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	// Create buffers if they don't exist
	/*if ((unsigned int)depth1_.width() != camera.width || (unsigned int)depth1_.height() != camera.height) {
		depth1_ = ftl::cuda::TextureObject<int>(camera.width, camera.height);
	}
	if ((unsigned int)depth3_.width() != camera.width || (unsigned int)depth3_.height() != camera.height) {
		depth3_ = ftl::cuda::TextureObject<int>(camera.width, camera.height);
	}
	if ((unsigned int)colour1_.width() != camera.width || (unsigned int)colour1_.height() != camera.height) {
		colour1_ = ftl::cuda::TextureObject<uchar4>(camera.width, camera.height);
	}
	if ((unsigned int)colour_tmp_.width() != camera.width || (unsigned int)colour_tmp_.height() != camera.height) {
		colour_tmp_ = ftl::cuda::TextureObject<float4>(camera.width, camera.height);
	}
	if ((unsigned int)normal1_.width() != camera.width || (unsigned int)normal1_.height() != camera.height) {
		normal1_ = ftl::cuda::TextureObject<float4>(camera.width, camera.height);
	}
	if ((unsigned int)depth2_.width() != camera.width || (unsigned int)depth2_.height() != camera.height) {
		depth2_ = ftl::cuda::TextureObject<float>(camera.width, camera.height);
	}
	if ((unsigned int)colour2_.width() != camera.width || (unsigned int)colour2_.height() != camera.height) {
		colour2_ = ftl::cuda::TextureObject<uchar4>(camera.width, camera.height);
	}*/

	// Parameters object to pass to CUDA describing the camera
	SplatParams params;
	params.m_flags = 0;
	if (src->value("splatting", true) == false) params.m_flags |= ftl::render::kNoSplatting;
	if (src->value("upsampling", true) == false) params.m_flags |= ftl::render::kNoUpsampling;
	if (src->value("texturing", true) == false) params.m_flags |= ftl::render::kNoTexturing;

	params.m_viewMatrix = MatrixConversion::toCUDA(src->getPose().cast<float>().inverse());
	params.m_viewMatrixInverse = MatrixConversion::toCUDA(src->getPose().cast<float>());
	//params.voxelSize = scene_->getHashParams().m_virtualVoxelSize;
	params.camera = camera;
	/*params.camera.fx = camera.fx;
	params.camera.fy = camera.fy;
	params.camera.mx = -camera.cx;
	params.camera.my = -camera.cy;
	params.camera.m_imageWidth = camera.width;
	params.camera.m_imageHeight = camera.height;
	params.camera.m_sensorDepthWorldMax = camera.maxDepth;
	params.camera.m_sensorDepthWorldMin = camera.minDepth;*/

	//ftl::cuda::compactifyAllocated(scene_->getHashData(), scene_->getHashParams(), stream);
	//LOG(INFO) << "Occupied: " << scene_->getOccupiedCount();

	//if (scene_->value("voxels", false)) {
		// TODO:(Nick) Stereo for voxel version
		//ftl::cuda::isosurface_point_image(scene_->getHashData(), depth1_, params, stream);
		//ftl::cuda::splat_points(depth1_, depth2_, params, stream);
		//ftl::cuda::dibr(depth2_, colour1_, scene_->cameraCount(), params, stream);
		//src->writeFrames(ts, colour1_, depth2_, stream);
	//} else {

		//ftl::cuda::clear_depth(depth1_, stream);
		//ftl::cuda::clear_depth(depth3_, stream);
		//ftl::cuda::clear_depth(depth2_, stream);
		//ftl::cuda::clear_colour(colour2_, stream);
		
		temp_.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(0), cvstream);
		temp_.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0), cvstream);
		output_.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f), cvstream);
		output_.get<GpuMat>(Channel::Colour).setTo(cv::Scalar(0,0,0), cvstream);

		// Step 1: Put all points into virtual view to gather them
		//ftl::cuda::dibr_raw(depth1_, scene_->cameraCount(), params, stream);

		// Step 2: For each point, use a warp to do MLS and up sample
		//ftl::cuda::mls_render_depth(depth1_, depth3_, params, scene_->cameraCount(), stream);

		if (src->getChannel() == Channel::Depth) {
			//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
			if (value("splatting",  false)) {
				//ftl::cuda::splat_points(depth1_, colour1_, normal1_, depth2_, colour2_, params, stream);
				//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);

				temp_.get<GpuMat>(Channel::Depth).convertTo(output_.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 1000.0f, cvstream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				src->write(scene_.timestamp, output_, stream);
			} else {
				temp_.get<GpuMat>(Channel::Depth).convertTo(output_.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 1000.0f, cvstream);
				//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				src->write(scene_.timestamp, output_, stream);
			}
		} else if (src->getChannel() == Channel::Energy) {
			//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
			//if (src->value("splatting",  false)) {
				//ftl::cuda::splat_points(depth1_, colour1_, normal1_, depth2_, colour2_, params, stream);
				//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				src->write(scene_.timestamp, output_, stream);
			//} else {
				//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
			//	src->writeFrames(colour1_, depth2_, stream);
			//}
		} else if (src->getChannel() == Channel::Right) {
			// Adjust pose to right eye position
			Eigen::Affine3f transform(Eigen::Translation3f(camera.baseline,0.0f,0.0f));
			Eigen::Matrix4f matrix =  src->getPose().cast<float>() * transform.matrix();
			params.m_viewMatrix = MatrixConversion::toCUDA(matrix.inverse());
			params.m_viewMatrixInverse = MatrixConversion::toCUDA(matrix);

			//ftl::cuda::clear_depth(depth1_, stream);
			ftl::cuda::dibr(depth1_, colour1_, normal1_, depth2_, colour_tmp_, depth3_, scene_->cameraCount(), params, stream);
			//src->writeFrames(ts, colour1_, colour2_, stream);
			src->write(scene_.timestamp, output_, stream);
		} else {
			if (value("splatting",  false)) {
				//ftl::cuda::splat_points(depth1_, colour1_, normal1_, depth2_, colour2_, params, stream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				src->write(scene_.timestamp, output_, stream);
			} else {
				//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
				temp_.get<GpuMat>(Channel::Depth).convertTo(output_.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 1000.0f, cvstream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				src->write(scene_.timestamp, output_, stream);
			}
		}
	//}

	//ftl::cuda::median_filter(depth1_, depth2_, stream);
	//ftl::cuda::splat_points(depth1_, depth2_, params, stream);

	// TODO: Second pass

	return true;
}

void Splatter::setOutputDevice(int device) {
	device_ = device;
}
