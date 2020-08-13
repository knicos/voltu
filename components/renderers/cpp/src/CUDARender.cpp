#include <ftl/render/CUDARender.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include "splatter_cuda.hpp"
#include <ftl/cuda/points.hpp>
#include <ftl/cuda/normals.hpp>
#include <ftl/operators/cuda/mask.hpp>
#include <ftl/render/colouriser.hpp>
#include <ftl/cuda/transform.hpp>

#include <ftl/cuda/colour_cuda.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudaarithm.hpp>

//#include <ftl/filters/smoothing.hpp>

#include <string>

using ftl::render::CUDARender;
using ftl::codecs::Channel;
using ftl::codecs::Channels;
using ftl::rgbd::Format;
using ftl::rgbd::VideoFrame;
using cv::cuda::GpuMat;
using std::stoul;
using ftl::cuda::Mask;
using ftl::render::parseCUDAColour;
using ftl::render::parseCVColour;

CUDARender::CUDARender(nlohmann::json &config) : ftl::render::FSRenderer(config), temp_d_(ftl::data::Frame::make_standalone()), temp_(temp_d_.cast<ftl::rgbd::Frame>()), scene_(nullptr) {
	/*if (config["clipping"].is_object()) {
		auto &c = config["clipping"];
		float rx = c.value("pitch", 0.0f);
		float ry = c.value("yaw", 0.0f);
		float rz = c.value("roll", 0.0f);
		float x = c.value("x", 0.0f);
		float y = c.value("y", 0.0f);
		float z = c.value("z", 0.0f);
		float width = c.value("width", 1.0f);
		float height = c.value("height", 1.0f);
		float depth = c.value("depth", 1.0f);

		Eigen::Affine3f r = create_rotation_matrix(rx, ry, rz).cast<float>();
		Eigen::Translation3f trans(Eigen::Vector3f(x,y,z));
		Eigen::Affine3f t(trans);

		clip_.origin = MatrixConversion::toCUDA(r.matrix() * t.matrix());
		clip_.size = make_float3(width, height, depth);
		clipping_ = value("clipping_enabled", true);
	} else {*/
		clipping_ = false;
	//}

	params_.viewPortMode = ftl::render::ViewPortMode::Disabled;

	colouriser_ = ftl::create<ftl::render::Colouriser>(this, "colouriser");

	on("touch_sensitivity", touch_dist_, 0.04f);

	on("clipping_enabled", [this]() {
		clipping_ = value("clipping_enabled", true);
	});

	norm_filter_ = value("normal_filter", -1.0f);
	on("normal_filter", [this]() {
		norm_filter_ = value("normal_filter", -1.0f);
	});

	backcull_ = value("back_cull", true);
	on("back_cull", [this]() {
		backcull_ = value("back_cull", true);
	});

	mesh_ = value("meshing", true);
	on("meshing", [this]() {
		mesh_ = value("meshing", true);
	});

	background_ = parseCVColour(value("background", std::string("#4c4c4c")));
	on("background", [this]() {
		background_ = parseCVColour(value("background", std::string("#4c4c4c")));
	});

	// Load any environment texture
	std::string envimage = value("environment", std::string(""));
	if (envimage.size() > 0) {
		cv::Mat envim = cv::imread(envimage);
		if (!envim.empty()) {
			if (envim.type() == CV_8UC3) {
				cv::cvtColor(envim,envim, cv::COLOR_BGR2BGRA);
			}
			env_image_.upload(envim);
			env_tex_ = std::move(ftl::cuda::TextureObject<uchar4>(env_image_, true));
		}
	}

	cudaSafeCall(cudaStreamCreate(&stream_));
	last_frame_ = -1;

	temp_.store();
	// Allocate collisions buffer
	cudaSafeCall(cudaMalloc(&collisions_, 1024*sizeof(ftl::cuda::Collision)));
}

CUDARender::~CUDARender() {
	delete colouriser_;
	cudaFree(collisions_);
}

void CUDARender::_renderChannel(ftl::rgbd::Frame &output, ftl::codecs::Channel in, const Eigen::Matrix4d &t, cudaStream_t stream) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	if (in == Channel::None) return;

	for (size_t i=0; i < scene_->frames.size(); ++i) {
		//if (!scene_->hasFrame(i)) continue;
		auto &f = scene_->frames[i].cast<ftl::rgbd::Frame>();

		if (!f.hasChannel(in)) {
			//LOG(ERROR) << "Reprojecting unavailable channel";
			continue;
		}

		_adjustDepthThresholds(f.getLeftCamera());

		// Generate a colour texture from any channel
		auto &texture = colouriser_->colourise(f, in, stream);

		auto transform = MatrixConversion::toCUDA(f.getPose().cast<float>().inverse() * t.cast<float>().inverse()) * poseInverse_;
		auto transformR = MatrixConversion::toCUDA(f.getPose().cast<float>().inverse() * t.cast<float>().inverse()).getFloat3x3();

		if (f.hasChannel(Channel::Depth)) {
			ftl::cuda::reproject(
				texture,
				f.createTexture<float>(Channel::Depth),
				output.getTexture<float>(_getDepthChannel()),
				f.createTexture<short>(Channel::Weights),
				(mesh_) ? &output.getTexture<half4>(_getNormalsChannel()) : nullptr,
				accum_,
				contrib_,
				params_,
				f.getLeftCamera(),
				transform, transformR, stream
			);
		} else if (f.hasChannel(Channel::GroundTruth)) {
			ftl::cuda::reproject(
				texture,
				f.createTexture<float>(Channel::GroundTruth),
				output.getTexture<float>(_getDepthChannel()),
				f.createTexture<short>(Channel::Weights),
				(mesh_) ? &output.getTexture<half4>(_getNormalsChannel()) : nullptr,
				accum_,
				contrib_,
				params_,
				f.getLeftCamera(),
				transform, transformR, stream
			);
		} else {
			// Reproject without depth channel or normals
			ftl::cuda::reproject(
				texture,
				output.getTexture<float>(_getDepthChannel()),
				accum_,
				contrib_,
				params_,
				f.getLeftCamera(),
				transform, stream
			);
		}
	}
}

void CUDARender::_dibr(ftl::rgbd::Frame &out, const Eigen::Matrix4d &t, cudaStream_t stream) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	temp_.set<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0x7FFFFFFF), cvstream);

	for (size_t i=0; i < scene_->frames.size(); ++i) {
		if (!scene_->hasFrame(i)) continue;
		auto &f = scene_->frames[i].cast<ftl::rgbd::Frame>();
		//auto *s = scene_->sources[i];

		if (!f.has(Channel::Colour)) {
			LOG(ERROR) << "Missing required channel";
			continue;
		}

		auto transform = pose_ * MatrixConversion::toCUDA(t.cast<float>() * f.getPose().cast<float>());

		if (f.hasChannel(Channel::Depth)) {
			ftl::cuda::dibr_merge(
				f.createTexture<float>(Channel::Depth),
				temp_.createTexture<int>(Channel::Depth2),
				transform,
				f.getLeftCamera(),
				params_, stream
			);
		} else {
			ftl::cuda::dibr_merge(
				temp_.createTexture<int>(Channel::Depth2),
				transform,
				f.getLeftCamera(),
				params_, stream
			);
		}
	}

	// Convert from int depth to float depth
	temp_.get<GpuMat>(Channel::Depth2).convertTo(out.get<GpuMat>(_getDepthChannel()), CV_32F, 1.0f / 100000.0f, cvstream);
}

void CUDARender::_adjustDepthThresholds(const ftl::rgbd::Camera &fcam) {
	params_.depthCoef = fcam.baseline*fcam.fx;
}

ftl::cuda::TextureObject<float> &CUDARender::_getDepthBuffer(const cv::Size &size) {
	for (auto *b : depth_buffers_) {
		if (b->width() == static_cast<size_t>(size.width) && b->height() == static_cast<size_t>(size.height)) return *b;
	}
	auto *nb = new ftl::cuda::TextureObject<float>(size.width, size.height);
	depth_buffers_.push_back(nb);
	return *nb;
}

ftl::cuda::TextureObject<short2> &CUDARender::_getScreenBuffer(const cv::Size &size) {
	for (auto *b : screen_buffers_) {
		if (b->width() == static_cast<size_t>(size.width) && b->height() == static_cast<size_t>(size.height)) return *b;
	}
	auto *nb = new ftl::cuda::TextureObject<short2>(size.width, size.height);
	screen_buffers_.push_back(nb);
	return *nb;
}

void CUDARender::_mesh(ftl::rgbd::Frame &out, const Eigen::Matrix4d &t, cudaStream_t stream) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	bool do_blend = value("mesh_blend", false);
	float blend_alpha = value("blend_alpha", 0.02f);
	if (do_blend) {
		temp_.set<GpuMat>(Channel::Depth).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
		temp_.set<GpuMat>(Channel::Weights).setTo(cv::Scalar(0.0f), cvstream);
	} else {
		temp_.set<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
	}

	int valid_count = 0;

	// For each source depth map
	for (size_t i=0; i < scene_->frames.size(); ++i) {
		//if (!scene_->hasFrame(i)) continue;
		auto &f = scene_->frames[i].cast<ftl::rgbd::Frame>();
		//auto *s = scene_->sources[i];

		if (!f.has(Channel::Colour)) {
			//LOG(ERROR) << "Missing required channel";
			continue;
		}

		++valid_count;

		//auto pose = MatrixConversion::toCUDA(t.cast<float>() * f.getPose().cast<float>());
		auto transform = pose_ * MatrixConversion::toCUDA(t.cast<float>() * f.getPose().cast<float>());

		_adjustDepthThresholds(f.getLeftCamera());

		auto bufsize = f.get<GpuMat>((f.hasChannel(Channel::Depth)) ? Channel::Depth : Channel::Colour).size();
		auto &depthbuffer = _getDepthBuffer(bufsize);
		auto &screenbuffer = _getScreenBuffer(bufsize);

		// Calculate and save virtual view screen position of each source pixel
		if (f.hasChannel(Channel::Depth)) {
			LOG(INFO) << "DEPTH SCREEN COORD";
			ftl::cuda::screen_coord(
				f.createTexture<float>(Channel::Depth),
				depthbuffer,
				screenbuffer,
				params_, transform, f.getLeftCamera(), stream
			);
		} else if (f.hasChannel(Channel::GroundTruth)) {
			ftl::cuda::screen_coord(
				f.createTexture<float>(Channel::GroundTruth),
				depthbuffer,
				screenbuffer,
				params_, transform, f.getLeftCamera(), stream
			);
		} else {
			LOG(INFO) << "NON-DEPTH SCREEN COORD";
			// Constant depth version
			ftl::cuda::screen_coord(
				depthbuffer,
				screenbuffer,
				params_, transform, f.getLeftCamera(), stream
			);
		}

		// Must reset depth channel if blending
		if (do_blend) {
			temp_.set<GpuMat>(Channel::Depth).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
		}

		depth_out_.to_gpumat().setTo(cv::Scalar(1000.0f), cvstream);

		LOG(INFO) << "MESH BEFORE TRIANGLES";
		cudaSafeCall(cudaStreamSynchronize(stream_));

		// Decide on and render triangles around each point
		ftl::cuda::triangle_render1(
			depthbuffer,
			temp_.createTexture<int>((do_blend) ? Channel::Depth : Channel::Depth2),
			screenbuffer,
			params_, stream
		);

		// TODO: Reproject here
		// And merge based upon weight adjusted distances

		LOG(INFO) << "MESH BLENDER";
		cudaSafeCall(cudaStreamSynchronize(stream_));

		if (do_blend) {
			// Blend this sources mesh with previous meshes
			ftl::cuda::mesh_blender(
				temp_.getTexture<int>(Channel::Depth),
				//out.createTexture<float>(_getDepthChannel()),
				depth_out_,
				f.createTexture<short>(Channel::Weights),
				temp_.createTexture<float>(Channel::Weights),
				params_,
				f.getLeftCamera(),
				transform.getInverse(),
				blend_alpha,
				stream
			);
		}
	}

	if (valid_count == 0) return;

	LOG(INFO) << "MESH PART 1";
	cudaSafeCall(cudaStreamSynchronize(stream_));

	// Convert from int depth to float depth
	//temp_.get<GpuMat>(Channel::Depth2).convertTo(out.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 100000.0f, cvstream);

	if (do_blend) {
		ftl::cuda::dibr_normalise(
			//out.getTexture<float>(_getDepthChannel()),
			//out.getTexture<float>(_getDepthChannel()),
			depth_out_,
			depth_out_,
			temp_.getTexture<float>(Channel::Weights),
			stream_
		);
	} else {
		//ftl::cuda::merge_convert_depth(temp_.getTexture<int>(Channel::Depth2), out.createTexture<float>(_getDepthChannel()), 1.0f / 100000.0f, stream_);
		ftl::cuda::merge_convert_depth(temp_.getTexture<int>(Channel::Depth2), depth_out_, 1.0f / 100000.0f, stream_);
	}

	// Now merge new render to any existing frameset render, detecting collisions
	ftl::cuda::touch_merge(depth_out_, out.createTexture<float>(_getDepthChannel()), collisions_, 1024, touch_dist_, stream_);

	//filters_->filter(out, src, stream);

	// Generate normals for final virtual image
	ftl::cuda::normals(
		out.createTexture<half4>(_getNormalsChannel()),
		temp_.createTexture<half4>(Channel::Normals),
		out.getTexture<float>(_getDepthChannel()),
		value("normal_radius", 1), value("normal_smoothing", 0.02f),
		params_.camera, pose_.getFloat3x3(), poseInverse_.getFloat3x3(), stream_);
}

void CUDARender::_allocateChannels(ftl::rgbd::Frame &out, ftl::codecs::Channel chan) {
	const auto &camera = params_.camera;
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	// Allocate left channel buffers and clear them
	if (chan == Channel::Colour) {
		//if (!out.hasChannel(Channel::Depth)) {
			out.create<VideoFrame>(Channel::Depth).createGPU(Format<float>(camera.width, camera.height));
			out.create<VideoFrame>(Channel::Colour).createGPU(Format<uchar4>(camera.width, camera.height));
			out.create<VideoFrame>(Channel::Normals).createGPU(Format<half4>(camera.width, camera.height));
			out.createTexture<uchar4>(Channel::Colour, ftl::rgbd::Format<uchar4>(camera.width, camera.height), true);  // Force interpolated colour
			out.set<GpuMat>(Channel::Depth).setTo(cv::Scalar(1000.0f), cvstream);
		//}
	// Allocate right channel buffers and clear them
	} else {
		if (!out.hasChannel(Channel::Depth2)) {
			out.create<VideoFrame>(Channel::Depth2).createGPU(Format<float>(camera.width, camera.height));
			out.create<VideoFrame>(Channel::Colour2).createGPU(Format<uchar4>(camera.width, camera.height));
			out.create<VideoFrame>(Channel::Normals2).createGPU(Format<half4>(camera.width, camera.height));
			out.createTexture<uchar4>(Channel::Colour2, ftl::rgbd::Format<uchar4>(camera.width, camera.height), true);  // Force interpolated colour
			out.set<GpuMat>(Channel::Depth2).setTo(cv::Scalar(1000.0f), cvstream);
		}
	}
	
	temp_.create<VideoFrame>(Channel::Depth).createGPU(Format<int>(camera.width, camera.height));
	temp_.create<VideoFrame>(Channel::Depth2).createGPU(Format<int>(camera.width, camera.height));
	temp_.create<VideoFrame>(Channel::Normals).createGPU(Format<half4>(camera.width, camera.height));
	temp_.create<VideoFrame>(Channel::Weights).createGPU(Format<float>(camera.width, camera.height));
	temp_.createTexture<int>(Channel::Depth);

	depth_out_.create(camera.width, camera.height);
	accum_.create(camera.width, camera.height);
	contrib_.create(camera.width, camera.height);

	// Must clear the colour accumulation buffers each frame
	cv::cuda::GpuMat accum(accum_.height(), accum_.width(), CV_32FC4, accum_.devicePtr(), accum_.pitch());
	accum.setTo(cv::Scalar(0.0f,0.0f,0.0f,0.0f), cvstream);
	cv::cuda::GpuMat contrib(contrib_.height(), contrib_.width(), CV_32S, contrib_.devicePtr(), contrib_.pitch());
	contrib.setTo(cv::Scalar(0), cvstream);

	//normals_.create(camera.width, camera.height);
}

void CUDARender::_updateParameters(ftl::rgbd::Frame &out, ftl::codecs::Channel chan) {
	const auto &camera = (chan == Channel::Right) ? out.getRightCamera() : out.getLeftCamera();
	params_.camera = camera;

	if (chan == Channel::Right) {
		// Right channel needs to adjust pose by a baseline translation
		float baseline = params_.camera.baseline;
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
		transform(0, 3) = baseline;
		Eigen::Matrix4f matrix = out.getPose().cast<float>() * transform.inverse();
		pose_ = MatrixConversion::toCUDA(matrix.inverse());
		poseInverse_ = MatrixConversion::toCUDA(matrix);
	} else {
		poseInverse_ = MatrixConversion::toCUDA(out.getPose().cast<float>());
		pose_ = MatrixConversion::toCUDA(out.getPose().cast<float>().inverse());
	}

	// Parameters object to pass to CUDA describing the camera
	params_.triangle_limit = value("triangle_limit", 200);
	params_.disconDisparities = value("discon_disparities", 2.0f);
	params_.accumulationMode = static_cast<ftl::render::AccumulationFunction>(value("accumulation_func", 0));
	params_.m_flags = 0;
	params_.projection = static_cast<ftl::rgbd::Projection>(value("projection",0));
	if (value("normal_weight_colours", true)) params_.m_flags |= ftl::render::kNormalWeightColours;
	if (value("channel_weights", false)) params_.m_flags |= ftl::render::kUseWeightsChannel;
}

void CUDARender::_postprocessColours(ftl::rgbd::Frame &out) {
	if (value("cool_effect", false)) {
		auto pose = poseInverse_.getFloat3x3();
		auto col = parseCUDAColour(value("cool_effect_colour", std::string("#2222ff")));

		ftl::cuda::cool_blue(
			out.getTexture<half4>(_getNormalsChannel()),
			out.getTexture<uchar4>(out_chan_),
			col, pose,
			stream_
		);
	}

	if (value("show_colour_weights", false)) {
		ftl::cuda::show_colour_weights(
			out.getTexture<uchar4>(out_chan_),
			contrib_,
			make_uchar4(0,0,255,0),
			stream_
		);
	} else if (value("show_bad_colour", false)) {
		ftl::cuda::show_missing_colour(
			out.getTexture<float>(_getDepthChannel()),
			out.getTexture<uchar4>(out_chan_),
			contrib_,
			make_uchar4(255,0,0,0),
			params_.camera,
			stream_
		);
	} else if (mesh_ && out.hasChannel(_getDepthChannel()) && out.hasChannel(out_chan_)) {
		ftl::cuda::fix_bad_colour(
			out.getTexture<float>(_getDepthChannel()),
			out.getTexture<uchar4>(out_chan_),
			contrib_,
			make_uchar4(255,0,0,0),
			params_.camera,
			stream_
		);
	}
}

void CUDARender::_renderPass1(const Eigen::Matrix4d &t) {
	//const auto &camera = out_->getLeftCamera();
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	if (mesh_) {
		// Render depth channel using triangles
		_mesh(*out_, t, stream_);
	} else {
		// Render depth channel as a point cloud
		_dibr(*out_, t, stream_);
	}
}

void CUDARender::_renderPass2(Channels<0> chans, const Eigen::Matrix4d &t) {
	for (auto chan : chans) {
		ftl::codecs::Channel mapped = chan;

		// FIXME: Doesn't seem to work
		//if (chan == Channel::Colour && scene_->firstFrame().hasChannel(Channel::ColourHighRes)) mapped = Channel::ColourHighRes;

		_renderChannel(*out_, mapped, t, stream_);
	}
}

void CUDARender::cancel() {
	out_ = nullptr;
	scene_ = nullptr;
	stage_ = Stage::Finished;
	cudaSafeCall(cudaStreamSynchronize(stream_));
}

void CUDARender::begin(ftl::rgbd::Frame &out, ftl::codecs::Channel chan) {
	if (stage_ != Stage::Finished) {
		throw FTL_Error("Already rendering");
	}

	out_ = &out;
	const auto &camera = out.getLeftCamera();
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	if (chan != Channel::Colour && chan != Channel::Colour2) {
		throw FTL_Error("Can only render to Left or Right channels");
	}

	out_chan_ = chan;
	_updateParameters(out, chan);
	_allocateChannels(out, chan);

	// Apply a colour background
	if (env_image_.empty() || !value("environment_enabled", false)) {
		out.set<GpuMat>(chan).setTo(background_, cvstream);
	} else {
		auto pose = poseInverse_.getFloat3x3();
		ftl::cuda::equirectangular_reproject(
			env_tex_,
			out.createTexture<uchar4>(Channel::Colour, true),
			camera, pose, stream_);
	}

	sets_.clear();
	stage_ = Stage::ReadySubmit;

	// Reset collision data.
	cudaSafeCall(cudaMemsetAsync(collisions_, 0, sizeof(int), stream_));
}

void CUDARender::render() {
	if (stage_ != Stage::ReadySubmit) {
		throw FTL_Error("Cannot call render at this time");
	} else {
		stage_ = Stage::Blending;
		_endSubmit();
	}
}

void CUDARender::blend(Channel c) {
	if (stage_ != Stage::Blending) {
		throw FTL_Error("Cannot call blend at this time");
	}

	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	auto &buf = colouriser_->colourise(*out_, c, stream_);
	//cv::cuda::addWeighted(buf.to_gpumat(), alpha, out_->get<GpuMat>(out_chan_), 1.0f-alpha, 0.0f,
	//	out_->get<GpuMat>(out_chan_), -1, cvstream);

	//if (alpha < 0.0f) {
		ftl::cuda::composite(buf, out_->getTexture<uchar4>(out_chan_), stream_);
	//} else {
	//	ftl::cuda::blend_alpha(buf, out_->getTexture<uchar4>(out_chan_), alpha, 1.0f-alpha, stream_);
	//}
}

void CUDARender::end() {
	if (stage_ == Stage::ReadySubmit) {
		_endSubmit();
	}

	_end();
	stage_ = Stage::Finished;
}

void CUDARender::_endSubmit() {
	// Carry out all reprojections
	for (auto &s : sets_) {
		scene_ = s.fs;
		try {
			_renderPass2(s.channels, s.transform);
		} catch(std::exception &e) {
			LOG(ERROR) << "Exception in render: " << e.what();
		}
	}

	scene_ = nullptr;

	// Convert accumulated colour to BGRA 8bit
	ftl::cuda::dibr_normalise(
		accum_,
		out_->getTexture<uchar4>(out_chan_),
		contrib_,
		false,  // Flip
		stream_
	);
}

void CUDARender::_end() {
	_postprocessColours(*out_);

	// Final OpenGL flip (easier to do in shader?)
	/*ftl::cuda::flip(out_->getTexture<uchar4>(out_chan_), stream_);*/
	/*ftl::cuda::flip(out_->getTexture<float>(_getDepthChannel()), stream_);*/

	cudaSafeCall(cudaMemcpyAsync(collisions_host_, collisions_, sizeof(ftl::cuda::Collision)*1024, cudaMemcpyDeviceToHost, stream_));
	cudaSafeCall(cudaStreamSynchronize(stream_));

	// Convert collisions into camera coordinates.
	collision_points_.resize(collisions_host_[0].screen);
	for (uint i=1; i<collisions_host_[0].screen+1; ++i) {
		collision_points_[i-1] = make_float4(collisions_host_[i].x(), collisions_host_[i].y(), collisions_host_[i].depth, collisions_host_[i].strength());
	}

	// Do something with the collisions
	/*if (collisions_host_[0].screen > 0) {
		float x = 0.0f;
		float y = 0.0f;
		float d = 0.0f;
		float w = 0.0f;

		for (uint i=1; i<collisions_host_[0].screen+1; ++i) {
			float inum = collisions_host_[i].strength();
			int ix = collisions_host_[i].x();
			int iy = collisions_host_[i].y();
			x += float(ix)*inum;
			y += float(iy)*inum;
			d += collisions_host_[i].depth*inum;
			w += inum;
		}

		x /= w;
		y /= w;
		d /= w;

		LOG(INFO) << "Collision at: " << x << "," << y << ", " << d;
	}*/
}

bool CUDARender::submit(ftl::data::FrameSet *in, Channels<0> chans, const Eigen::Matrix4d &t) {
	if (stage_ != Stage::ReadySubmit) {
		throw FTL_Error("Renderer not ready for submits");
	}

	scene_ = in;
	if (scene_->frames.size() == 0) {
		scene_ = nullptr;
		return false;
	}

	bool success = true;

	try {
		LOG(INFO) << "START RENDERPASS 1";
		_renderPass1(t);
		cudaSafeCall(cudaStreamSynchronize(stream_));
		LOG(INFO) << "END RENDERPASS 1";
	} catch (const ftl::exception &e) {
		LOG(ERROR) << "Exception in render: " << e.what();
		success = false;
	}

	auto &s = sets_.emplace_back();
	s.fs = in;
	s.channels = chans;
	s.transform = t;

	last_frame_ = scene_->timestamp();
	scene_ = nullptr;
	return success;
}
