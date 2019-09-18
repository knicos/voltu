#include <ftl/render/splat_render.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include "splatter_cuda.hpp"
#include <ftl/cuda/points.hpp>

#include <opencv2/core/cuda_stream_accessor.hpp>

using ftl::render::Splatter;
using ftl::rgbd::Channel;
using ftl::rgbd::Channels;
using ftl::rgbd::Format;
using cv::cuda::GpuMat;

Splatter::Splatter(nlohmann::json &config, ftl::rgbd::FrameSet *fs) : ftl::render::Renderer(config), scene_(fs) {

}

Splatter::~Splatter() {

}

bool Splatter::render(ftl::rgbd::VirtualSource *src, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (!src->isReady()) return false;

	LOG(INFO) << "Render ready2";

	const auto &camera = src->parameters();

	//cudaSafeCall(cudaSetDevice(scene_->getCUDADevice()));

	// Create all the required channels
	out.create<GpuMat>(Channel::Depth, Format<float>(camera.width, camera.height));
	out.create<GpuMat>(Channel::Colour, Format<uchar4>(camera.width, camera.height));

	temp_.create<GpuMat>(Channel::Colour, Format<float4>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Colour2, Format<uchar4>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Confidence, Format<float>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth2, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Normals, Format<float4>(camera.width, camera.height));

	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	LOG(INFO) << "Render ready1";

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
	params.camera = camera;

	// Clear all channels to 0 or max depth
	temp_.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
	temp_.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
	out.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(1000.0f), cvstream);
	out.get<GpuMat>(Channel::Colour).setTo(cv::Scalar(0,0,0), cvstream);

	LOG(INFO) << "Render ready";

	// Render each camera into virtual view
	for (size_t i=0; i<scene_->frames.size(); ++i) {
		auto &f = scene_->frames[i];
		auto *s = scene_->sources[i];

		if (!f.hasChannel(Channel::Depth) || f.isCPU(Channel::Depth)) {
			LOG(ERROR) << "Missing required Depth channel";
			return false;
		}

		// Needs to create points channel first?
		if (!f.hasChannel(Channel::Points)) {
			LOG(INFO) << "Creating points...";
			
			auto &t = f.createTexture<float4>(Channel::Points, Format<float4>(f.get<GpuMat>(Channel::Colour).size()));
			auto pose = MatrixConversion::toCUDA(s->getPose().cast<float>().inverse());
			ftl::cuda::point_cloud(t, f.createTexture<float>(Channel::Depth), s->parameters(), pose, stream);

			LOG(INFO) << "POINTS Added";
		}

		ftl::cuda::dibr_merge(
			f.createTexture<float4>(Channel::Points),
			temp_.createTexture<int>(Channel::Depth),
			params, stream
		);

		LOG(INFO) << "DIBR DONE";
	}

		//ftl::cuda::dibr(depth1_, colour1_, normal1_, depth2_, colour_tmp_, depth3_, scene_->cameraCount(), params, stream);

		// Step 1: Put all points into virtual view to gather them
		//ftl::cuda::dibr_raw(depth1_, scene_->cameraCount(), params, stream);

		// Step 2: For each point, use a warp to do MLS and up sample
		//ftl::cuda::mls_render_depth(depth1_, depth3_, params, scene_->cameraCount(), stream);

		if (src->getChannel() == Channel::Depth) {
			//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
			if (value("splatting",  false)) {
				//ftl::cuda::splat_points(depth1_, colour1_, normal1_, depth2_, colour2_, params, stream);
				//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);

				temp_.get<GpuMat>(Channel::Depth).convertTo(out.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 1000.0f, cvstream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				//src->write(scene_.timestamp, output_, stream);
			} else {
				temp_.get<GpuMat>(Channel::Depth).convertTo(out.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 1000.0f, cvstream);
				//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				//src->write(scene_.timestamp, output_, stream);
			}
		} else if (src->getChannel() == Channel::Energy) {
			//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
			//if (src->value("splatting",  false)) {
				//ftl::cuda::splat_points(depth1_, colour1_, normal1_, depth2_, colour2_, params, stream);
				//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				//src->write(scene_.timestamp, output_, stream);
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
			//ftl::cuda::dibr(depth1_, colour1_, normal1_, depth2_, colour_tmp_, depth3_, scene_->cameraCount(), params, stream);
			//src->writeFrames(ts, colour1_, colour2_, stream);
			//src->write(scene_.timestamp, output_, stream);
		} else {
			if (value("splatting",  false)) {
				//ftl::cuda::splat_points(depth1_, colour1_, normal1_, depth2_, colour2_, params, stream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				//src->write(scene_.timestamp, out, stream);
			} else {
				//ftl::cuda::int_to_float(depth1_, depth2_, 1.0f / 1000.0f, stream);
				temp_.get<GpuMat>(Channel::Depth).convertTo(out.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 1000.0f, cvstream);
				//src->writeFrames(ts, colour1_, depth2_, stream);
				//src->write(scene_.timestamp, output_, stream);
			}
		}
	//}

	//ftl::cuda::median_filter(depth1_, depth2_, stream);
	//ftl::cuda::splat_points(depth1_, depth2_, params, stream);

	// TODO: Second pass

	return true;
}

//void Splatter::setOutputDevice(int device) {
//	device_ = device;
//}
