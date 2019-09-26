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

void Splatter::renderChannel(
					SplatParams &params, ftl::rgbd::Frame &out,
					const Channel &channel, cudaStream_t stream)
{
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	temp_.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
	temp_.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
	temp_.get<GpuMat>(Channel::Colour).setTo(cv::Scalar(0.0f,0.0f,0.0f,0.0f), cvstream);
	temp_.get<GpuMat>(Channel::Contribution).setTo(cv::Scalar(0.0f), cvstream);
	
	// Render each camera into virtual view
	for (size_t i=0; i < scene_->frames.size(); ++i) {
		auto &f = scene_->frames[i];
		auto *s = scene_->sources[i];

		if (f.empty(Channel::Depth + Channel::Colour)) {
			LOG(ERROR) << "Missing required channel";
			continue;
		}

		// Needs to create points channel first?
		if (!f.hasChannel(Channel::Points)) {
			//LOG(INFO) << "Creating points... " << s->parameters().width;
			
			auto &t = f.createTexture<float4>(Channel::Points, Format<float4>(f.get<GpuMat>(Channel::Colour).size()));
			auto pose = MatrixConversion::toCUDA(s->getPose().cast<float>()); //.inverse());
			ftl::cuda::point_cloud(t, f.createTexture<float>(Channel::Depth), s->parameters(), pose, stream);

			//LOG(INFO) << "POINTS Added";
		}

		ftl::cuda::dibr_merge(
			f.createTexture<float4>(Channel::Points),
			temp_.getTexture<int>(Channel::Depth),
			params, stream
		);

		//LOG(INFO) << "DIBR DONE";
	}

	// TODO: Add the depth splatting step..

	temp_.createTexture<float4>(Channel::Colour);
	temp_.createTexture<float>(Channel::Contribution);

	// Accumulate attribute contributions for each pixel
	for (auto &f : scene_->frames) {
		// Convert colour from BGR to BGRA if needed
		if (f.get<GpuMat>(Channel::Colour).type() == CV_8UC3) {
			// Convert to 4 channel colour
			auto &col = f.get<GpuMat>(Channel::Colour);
			GpuMat tmp(col.size(), CV_8UC4);
			cv::cuda::swap(col, tmp);
			cv::cuda::cvtColor(tmp,col, cv::COLOR_BGR2BGRA);
		}
	
		ftl::cuda::dibr_attribute(
			f.createTexture<uchar4>(Channel::Colour),
			f.createTexture<float4>(Channel::Points),
			temp_.getTexture<int>(Channel::Depth),
			temp_.getTexture<float4>(Channel::Colour),
			temp_.getTexture<float>(Channel::Contribution),
			params, stream
		);
	}

	// Normalise attribute contributions
	ftl::cuda::dibr_normalise(
		temp_.createTexture<float4>(Channel::Colour),
		out.createTexture<uchar4>(channel),
		temp_.createTexture<float>(Channel::Contribution),
		stream
	);
}

bool Splatter::render(ftl::rgbd::VirtualSource *src, ftl::rgbd::Frame &out, cudaStream_t stream) {
	SHARED_LOCK(scene_->mtx, lk);
	if (!src->isReady()) return false;

	const auto &camera = src->parameters();

	//cudaSafeCall(cudaSetDevice(scene_->getCUDADevice()));

	// Create all the required channels
	out.create<GpuMat>(Channel::Depth, Format<float>(camera.width, camera.height));
	out.create<GpuMat>(Channel::Colour, Format<uchar4>(camera.width, camera.height));

	// FIXME: Use source resolutions, not virtual resolution
	temp_.create<GpuMat>(Channel::Colour, Format<float4>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Colour2, Format<uchar4>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Contribution, Format<float>(camera.width, camera.height));
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
	params.camera = camera;

	// Clear all channels to 0 or max depth

	out.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(1000.0f), cvstream);
	out.get<GpuMat>(Channel::Colour).setTo(cv::Scalar(76,76,76), cvstream);

	//LOG(INFO) << "Render ready: " << camera.width << "," << camera.height;

	temp_.createTexture<int>(Channel::Depth);

	renderChannel(params, out, Channel::Colour, stream);
	
	Channel chan = src->getChannel();
	if (chan == Channel::Depth)
	{
		temp_.get<GpuMat>(Channel::Depth).convertTo(out.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 1000.0f, cvstream);
	}
	else if (chan == Channel::Energy)
	{
		cv::cuda::swap(temp_.get<GpuMat>(Channel::Energy), out.create<GpuMat>(Channel::Energy));
	}
	else if (chan == Channel::Right)
	{
		Eigen::Affine3f transform(Eigen::Translation3f(camera.baseline,0.0f,0.0f));
		Eigen::Matrix4f matrix =  src->getPose().cast<float>() * transform.matrix();
		params.m_viewMatrix = MatrixConversion::toCUDA(matrix.inverse());
		params.m_viewMatrixInverse = MatrixConversion::toCUDA(matrix);
		
		out.create<GpuMat>(Channel::Right, Format<uchar4>(camera.width, camera.height));
		out.get<GpuMat>(Channel::Right).setTo(cv::Scalar(76,76,76), cvstream);
		renderChannel(params, out, Channel::Right, stream);
		cv::imshow("RIGHT", out.get<cv::Mat>(Channel::Right));
		cv::waitKey(1);
	}

	return true;
}

//void Splatter::setOutputDevice(int device) {
//	device_ = device;
//}
