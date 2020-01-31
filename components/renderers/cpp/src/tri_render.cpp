#include <ftl/render/tri_render.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include "splatter_cuda.hpp"
#include <ftl/cuda/points.hpp>
#include <ftl/cuda/normals.hpp>
#include <ftl/cuda/mask.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>

//#include <ftl/filters/smoothing.hpp>

#include <string>

using ftl::render::Triangular;
using ftl::codecs::Channel;
using ftl::codecs::Channels;
using ftl::rgbd::Format;
using cv::cuda::GpuMat;
using std::stoul;
using ftl::cuda::Mask;

static Eigen::Affine3d create_rotation_matrix(float ax, float ay, float az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * rx * ry;
}

/*
 * Parse a CSS style colour string into a scalar.
 */
static cv::Scalar parseCVColour(const std::string &colour) {
	std::string c = colour;
	if (c[0] == '#') {
		c.erase(0, 1);
		unsigned long value = stoul(c.c_str(), nullptr, 16);
		return cv::Scalar(
			(value >> 0) & 0xff,
			(value >> 8) & 0xff,
			(value >> 16) & 0xff,
			(value >> 24) & 0xff
		);
	}

	return cv::Scalar(0,0,0,0);
}

/*
 * Parse a CSS style colour string into a scalar.
 */
static uchar4 parseCUDAColour(const std::string &colour) {
	std::string c = colour;
	if (c[0] == '#') {
		c.erase(0, 1);
		unsigned long value = stoul(c.c_str(), nullptr, 16);
		return make_uchar4(
			(value >> 0) & 0xff,
			(value >> 8) & 0xff,
			(value >> 16) & 0xff,
			(value >> 24) & 0xff
		);
	}

	return make_uchar4(0,0,0,0);
}

Triangular::Triangular(nlohmann::json &config) : ftl::render::Renderer(config), scene_(nullptr) {
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

	on("clipping_enabled", [this](const ftl::config::Event &e) {
		clipping_ = value("clipping_enabled", true);
	});

	norm_filter_ = value("normal_filter", -1.0f);
	on("normal_filter", [this](const ftl::config::Event &e) {
		norm_filter_ = value("normal_filter", -1.0f);
	});

	backcull_ = value("back_cull", true);
	on("back_cull", [this](const ftl::config::Event &e) {
		backcull_ = value("back_cull", true);
	});

	mesh_ = value("meshing", true);
	on("meshing", [this](const ftl::config::Event &e) {
		mesh_ = value("meshing", true);
	});

	background_ = parseCVColour(value("background", std::string("#4c4c4c")));
	on("background", [this](const ftl::config::Event &e) {
		background_ = parseCVColour(value("background", std::string("#4c4c4c")));
	});

	light_diffuse_ = parseCUDAColour(value("diffuse", std::string("#e0e0e0")));
	on("diffuse", [this](const ftl::config::Event &e) {
		light_diffuse_ = parseCUDAColour(value("diffuse", std::string("#e0e0e0")));
	});

	light_ambient_ = parseCUDAColour(value("ambient", std::string("#0e0e0e")));
	on("ambient", [this](const ftl::config::Event &e) {
		light_ambient_ = parseCUDAColour(value("ambient", std::string("#0e0e0e")));
	});

	light_pos_ = make_float3(value("light_x", 0.3f), value("light_y", 0.2f), value("light_z", 1.0f));
	on("light_x", [this](const ftl::config::Event &e) { light_pos_.x = value("light_x", 0.3f); });
	on("light_y", [this](const ftl::config::Event &e) { light_pos_.y = value("light_y", 0.3f); });
	on("light_z", [this](const ftl::config::Event &e) { light_pos_.z = value("light_z", 0.3f); });

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

	//filters_ = ftl::create<ftl::Filters>(this, "filters");
	//filters_->create<ftl::filters::DepthSmoother>("hfnoise");
	last_frame_ = -1;
}

Triangular::~Triangular() {

}

template <typename T>
struct AccumSelector {
	typedef float4 type;
	static constexpr Channel channel = Channel::Colour;
	//static constexpr cv::Scalar value = cv::Scalar(0.0f,0.0f,0.0f,0.0f);
};

template <>
struct AccumSelector<float> {
	typedef float type;
	static constexpr Channel channel = Channel::Colour2;
	//static constexpr cv::Scalar value = cv::Scalar(0.0f);
};

/*template <typename T>
void Triangular::__blendChannel(ftl::rgbd::Frame &output, ftl::codecs::Channel in, ftl::codecs::Channel out, cudaStream_t stream) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	temp_.create<GpuMat>(
		AccumSelector<T>::channel,
		Format<typename AccumSelector<T>::type>(params_.camera.width, params_.camera.height)
	).setTo(cv::Scalar(0.0f), cvstream);
	temp_.get<GpuMat>(Channel::Contribution).setTo(cv::Scalar(0.0f), cvstream);

	temp_.createTexture<float>(Channel::Contribution);

	for (auto &f : scene_->frames) {
		if (f.get<GpuMat>(in).type() == CV_8UC3) {
			// Convert to 4 channel colour
			auto &col = f.get<GpuMat>(in);
			GpuMat tmp(col.size(), CV_8UC4);
			cv::cuda::swap(col, tmp);
			cv::cuda::cvtColor(tmp,col, cv::COLOR_BGR2BGRA);
		}

		ftl::cuda::dibr_attribute(
			f.createTexture<T>(in),
			f.createTexture<float4>(Channel::Points),
			temp_.getTexture<int>(Channel::Depth2),
			temp_.createTexture<typename AccumSelector<T>::type>(AccumSelector<T>::channel),
			temp_.getTexture<float>(Channel::Contribution),
			params_, stream
		);
	}

	ftl::cuda::dibr_normalise(
		temp_.getTexture<typename AccumSelector<T>::type>(AccumSelector<T>::channel),
		output.createTexture<T>(out),
		temp_.getTexture<float>(Channel::Contribution),
		stream
	);
}*/

template <typename T>
void Triangular::__reprojectChannel(ftl::rgbd::Frame &output, ftl::codecs::Channel in, ftl::codecs::Channel out, const Eigen::Matrix4d &t, cudaStream_t stream) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	temp_.create<GpuMat>(
		AccumSelector<T>::channel,
		Format<typename AccumSelector<T>::type>(params_.camera.width, params_.camera.height)
	).setTo(cv::Scalar(0.0f), cvstream);
	temp_.get<GpuMat>(Channel::Contribution).setTo(cv::Scalar(0.0f), cvstream);

	temp_.createTexture<float>(Channel::Contribution);

	for (size_t i=0; i < scene_->frames.size(); ++i) {
		auto &f = scene_->frames[i];
		//auto *s = scene_->sources[i];
		
		/*if (f.get<GpuMat>(in).type() == CV_8UC3) {
			// Convert to 4 channel colour
			auto &col = f.get<GpuMat>(in);
			GpuMat tmp(col.size(), CV_8UC4);
			cv::cuda::swap(col, tmp);
			cv::cuda::cvtColor(tmp,col, cv::COLOR_BGR2BGRA);
		}*/

		if (!f.hasChannel(in)) {
			LOG(ERROR) << "Reprojecting unavailable channel";
			return;
		}

		auto transform = MatrixConversion::toCUDA(f.getPose().cast<float>().inverse() * t.cast<float>().inverse()) * params_.m_viewMatrixInverse;
		auto transformR = MatrixConversion::toCUDA(f.getPose().cast<float>().inverse()).getFloat3x3();

		if (mesh_) {
			ftl::cuda::reproject(
				f.createTexture<T>(in),
				f.createTexture<float>(Channel::Depth),
				output.getTexture<float>(Channel::Depth),
				output.getTexture<float4>(Channel::Normals),
				temp_.createTexture<typename AccumSelector<T>::type>(AccumSelector<T>::channel),
				temp_.getTexture<float>(Channel::Contribution),
				params_,
				f.getLeftCamera(),
				transform, transformR, stream
			);
		} else {
			// Can't use normals with point cloud version
			ftl::cuda::reproject(
				f.createTexture<T>(in),
				f.createTexture<float>(Channel::Depth),
				output.getTexture<float>(Channel::Depth),
				temp_.createTexture<typename AccumSelector<T>::type>(AccumSelector<T>::channel),
				temp_.getTexture<float>(Channel::Contribution),
				params_,
				f.getLeftCamera(),
				transform, stream
			);
		}
	}

	ftl::cuda::dibr_normalise(
		temp_.getTexture<typename AccumSelector<T>::type>(AccumSelector<T>::channel),
		output.createTexture<T>(out),
		temp_.getTexture<float>(Channel::Contribution),
		stream
	);
}

/*void Triangular::_blendChannel(ftl::rgbd::Frame &output, ftl::codecs::Channel in, ftl::codecs::Channel out, cudaStream_t stream) {
	int type = output.get<GpuMat>(out).type(); // == CV_32F; //ftl::rgbd::isFloatChannel(channel);
	
	switch (type) {
	case CV_32F		: __blendChannel<float>(output, in, out, stream); break;
	case CV_32FC4	: __blendChannel<float4>(output, in, out, stream); break;
	case CV_8UC4	: __blendChannel<uchar4>(output, in, out, stream); break;
	default			: LOG(ERROR) << "Invalid output channel format";
	}
}*/

void Triangular::_reprojectChannel(ftl::rgbd::Frame &output, ftl::codecs::Channel in, ftl::codecs::Channel out, const Eigen::Matrix4d &t, cudaStream_t stream) {
	int type = output.get<GpuMat>(out).type(); // == CV_32F; //ftl::rgbd::isFloatChannel(channel);
	
	switch (type) {
	case CV_32F		: __reprojectChannel<float>(output, in, out, t, stream); break;
	case CV_32FC4	: __reprojectChannel<float4>(output, in, out, t, stream); break;
	case CV_8UC4	: __reprojectChannel<uchar4>(output, in, out, t, stream); break;
	default			: LOG(ERROR) << "Invalid output channel format";
	}
}

void Triangular::_dibr(ftl::rgbd::Frame &out, const Eigen::Matrix4d &t, cudaStream_t stream) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	temp_.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0x7FFFFFFF), cvstream);

	for (size_t i=0; i < scene_->frames.size(); ++i) {
		auto &f = scene_->frames[i];
		//auto *s = scene_->sources[i];

		if (f.empty(Channel::Depth + Channel::Colour)) {
			LOG(ERROR) << "Missing required channel";
			continue;
		}

		auto transform = params_.m_viewMatrix * MatrixConversion::toCUDA(t.cast<float>() * f.getPose().cast<float>());

		ftl::cuda::dibr_merge(
			f.createTexture<float>(Channel::Depth),
			temp_.createTexture<int>(Channel::Depth2),
			transform,
			f.getLeftCamera(),
			params_, stream
		);
	}

	// Convert from int depth to float depth
	temp_.get<GpuMat>(Channel::Depth2).convertTo(out.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 100000.0f, cvstream);
}

void Triangular::_mesh(ftl::rgbd::Frame &out, const Eigen::Matrix4d &t, cudaStream_t stream) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	bool do_blend = value("mesh_blend", true);
	float blend_alpha = value("blend_alpha", 0.02f);
	if (do_blend) {
		temp_.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
		temp_.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
	} else {
		temp_.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
	}

	// For each source depth map
	for (size_t i=0; i < scene_->frames.size(); ++i) {
		auto &f = scene_->frames[i];
		//auto *s = scene_->sources[i];

		if (f.empty(Channel::Depth + Channel::Colour)) {
			LOG(ERROR) << "Missing required channel";
			continue;
		}

		//auto pose = MatrixConversion::toCUDA(t.cast<float>() * f.getPose().cast<float>());
		auto transform = params_.m_viewMatrix * MatrixConversion::toCUDA(t.cast<float>() * f.getPose().cast<float>());

		// Calculate and save virtual view screen position of each source pixel
		ftl::cuda::screen_coord(
			f.createTexture<float>(Channel::Depth),
			f.createTexture<float>(Channel::Depth2, Format<float>(f.get<GpuMat>(Channel::Depth).size())),
			f.createTexture<short2>(Channel::Screen, Format<short2>(f.get<GpuMat>(Channel::Depth).size())),
			params_, transform, f.getLeftCamera(), stream
		);

		// Must reset depth channel if blending
		if (do_blend) {
			temp_.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(0x7FFFFFFF), cvstream);
		}

		// Decide on and render triangles around each point
		ftl::cuda::triangle_render1(
			f.getTexture<float>(Channel::Depth2),
			temp_.createTexture<int>((do_blend) ? Channel::Depth : Channel::Depth2),
			f.getTexture<short2>(Channel::Screen),
			params_, stream
		);

		if (do_blend) {
			// Blend this sources mesh with previous meshes
			ftl::cuda::mesh_blender(
				temp_.getTexture<int>(Channel::Depth),
				temp_.createTexture<int>(Channel::Depth2),
				params_.camera,
				blend_alpha,
				stream
			);
		}
	}

	// Convert from int depth to float depth
	//temp_.get<GpuMat>(Channel::Depth2).convertTo(out.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 100000.0f, cvstream);
	ftl::cuda::merge_convert_depth(temp_.getTexture<int>(Channel::Depth2), out.createTexture<float>(Channel::Depth), 1.0f / 100000.0f, stream_);

	//filters_->filter(out, src, stream);

	// Generate normals for final virtual image
	ftl::cuda::normals(out.createTexture<float4>(Channel::Normals, Format<float4>(params_.camera.width, params_.camera.height)),
				temp_.createTexture<float4>(Channel::Normals),
				out.createTexture<float>(Channel::Depth),
				value("normal_radius", 1), value("normal_smoothing", 0.02f),
				params_.camera, params_.m_viewMatrix.getFloat3x3(), params_.m_viewMatrixInverse.getFloat3x3(), stream_);
}

void Triangular::_renderChannel(
		ftl::rgbd::Frame &out,
		Channel channel_in, Channel channel_out, const Eigen::Matrix4d &t, cudaStream_t stream)
{
	if (channel_out == Channel::None || channel_in == Channel::None) return;
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	if (scene_->frames.size() < 1) return;
	bool is_float = out.get<GpuMat>(channel_out).type() == CV_32F; //ftl::rgbd::isFloatChannel(channel);
	bool is_4chan = out.get<GpuMat>(channel_out).type() == CV_32FC4;


	temp_.createTexture<float4>(Channel::Colour);
	temp_.createTexture<float>(Channel::Contribution);

	// FIXME: Using colour 2 in this way seems broken since it is already used
	if (is_4chan) {
		accum_.create<GpuMat>(channel_out, Format<float4>(params_.camera.width, params_.camera.height));
		accum_.get<GpuMat>(channel_out).setTo(cv::Scalar(0.0f,0.0f,0.0f,0.0f), cvstream);
	} else if (is_float) {
		accum_.create<GpuMat>(channel_out, Format<float>(params_.camera.width, params_.camera.height));
		accum_.get<GpuMat>(channel_out).setTo(cv::Scalar(0.0f), cvstream);
	} else {
		accum_.create<GpuMat>(channel_out, Format<uchar4>(params_.camera.width, params_.camera.height));
		accum_.get<GpuMat>(channel_out).setTo(cv::Scalar(0,0,0,0), cvstream);
	}

	_reprojectChannel(out, channel_in, channel_out, t, stream);
}

/*
 * H(Hue): 0 - 360 degree (integer)
 * S(Saturation): 0 - 1.00 (double)
 * V(Value): 0 - 1.00 (double)
 * 
 * output[3]: Output, array size 3, int
 */
static cv::Scalar HSVtoRGB(int H, double S, double V) {
	double C = S * V;
	double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
	double m = V - C;
	double Rs, Gs, Bs;

	if(H >= 0 && H < 60) {
		Rs = C;
		Gs = X;
		Bs = 0;	
	}
	else if(H >= 60 && H < 120) {	
		Rs = X;
		Gs = C;
		Bs = 0;	
	}
	else if(H >= 120 && H < 180) {
		Rs = 0;
		Gs = C;
		Bs = X;	
	}
	else if(H >= 180 && H < 240) {
		Rs = 0;
		Gs = X;
		Bs = C;	
	}
	else if(H >= 240 && H < 300) {
		Rs = X;
		Gs = 0;
		Bs = C;	
	}
	else {
		Rs = C;
		Gs = 0;
		Bs = X;	
	}

	return cv::Scalar((Bs + m) * 255, (Gs + m) * 255, (Rs + m) * 255, 0);
}

void Triangular::_allocateChannels(ftl::rgbd::Frame &out) {
	const auto &camera = out.getLeftCamera();
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	// Only do this if not already done, allows for multiple render passes with
	// different framesets.
	if (!out.hasChannel(Channel::Depth)) {
		out.create<GpuMat>(Channel::Depth, Format<float>(camera.width, camera.height));
		out.create<GpuMat>(Channel::Colour, Format<uchar4>(camera.width, camera.height));
		out.createTexture<uchar4>(Channel::Colour, true);  // Force interpolated colour
		out.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(1000.0f), cvstream);
	}

	temp_.create<GpuMat>(Channel::Colour, Format<float4>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Contribution, Format<float>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth2, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Normals, Format<float4>(camera.width, camera.height));
	temp_.createTexture<int>(Channel::Depth);
}

void Triangular::_updateParameters(ftl::rgbd::Frame &out) {
	const auto &camera = out.getLeftCamera();

	// Parameters object to pass to CUDA describing the camera
	params_.triangle_limit = value("triangle_limit", 200);
	params_.depthThreshold = value("depth_threshold", 0.04f);
	params_.m_flags = 0;
	if (value("normal_weight_colours", true)) params_.m_flags |= ftl::render::kNormalWeightColours;
	params_.m_viewMatrix = MatrixConversion::toCUDA(out.getPose().cast<float>().inverse());
	params_.m_viewMatrixInverse = MatrixConversion::toCUDA(out.getPose().cast<float>());
	params_.camera = camera;
}

void Triangular::_preprocessColours() {
	bool show_discon = value("show_discontinuity_mask", false);
	bool show_fill = value("show_filled", false);
	bool colour_sources = value("colour_sources", false);

	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	// Display mask values or otherwise alter colour image
	for (size_t i=0; i<scene_->frames.size(); ++i) {
		auto &f = scene_->frames[i];

		if (colour_sources) {
			auto colour = HSVtoRGB(360 / scene_->frames.size() * i, 0.6, 0.85);
			f.get<GpuMat>(Channel::Colour).setTo(colour, cvstream);
		}

		if (f.hasChannel(Channel::Mask)) {
			if (show_discon) {
				ftl::cuda::show_mask(f.getTexture<uchar4>(Channel::Colour), f.getTexture<int>(Channel::Mask), Mask::kMask_Discontinuity, make_uchar4(0,0,255,255), stream_);
			}
			if (show_fill) {
				ftl::cuda::show_mask(f.getTexture<uchar4>(Channel::Colour), f.getTexture<int>(Channel::Mask), Mask::kMask_Filled, make_uchar4(0,255,0,255), stream_);
			}
		}

		// Force interpolated colour
		f.createTexture<uchar4>(Channel::Colour, true);
	}
}

void Triangular::_postprocessColours(ftl::rgbd::Frame &out) {
	if (value("cool_effect", false)) {
		auto pose = params_.m_viewMatrixInverse.getFloat3x3();
		auto col = parseCUDAColour(value("cool_effect_colour", std::string("#2222ff")));

		ftl::cuda::cool_blue(
			out.getTexture<float4>(Channel::Normals),
			out.getTexture<uchar4>(Channel::Colour),
			col, pose,
			stream_	
		);
	}

	if (value("show_bad_colour", false)) {
		ftl::cuda::show_missing_colour(
			out.getTexture<float>(Channel::Depth),
			out.getTexture<uchar4>(Channel::Colour),
			temp_.getTexture<float>(Channel::Contribution),
			make_uchar4(255,0,0,0),
			params_.camera,
			stream_
		);
	} else if (out.hasChannel(Channel::Depth) && out.hasChannel(Channel::Colour) && temp_.hasChannel(Channel::Contribution)) {
		ftl::cuda::fix_bad_colour(
			out.getTexture<float>(Channel::Depth),
			out.getTexture<uchar4>(Channel::Colour),
			temp_.getTexture<float>(Channel::Contribution),
			make_uchar4(255,0,0,0),
			params_.camera,
			stream_
		);
	}
}

void Triangular::_renderNormals(ftl::rgbd::Frame &out) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	// Visualise normals to RGBA
	out.create<GpuMat>(Channel::ColourNormals, Format<uchar4>(params_.camera.width, params_.camera.height)).setTo(cv::Scalar(0,0,0,0), cvstream);

	ftl::cuda::normal_visualise(out.getTexture<float4>(Channel::Normals), out.createTexture<uchar4>(Channel::ColourNormals),
			light_pos_,
			light_diffuse_,
			light_ambient_, stream_);
}

void Triangular::_renderDensity(ftl::rgbd::Frame &out, const Eigen::Matrix4d &t) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);
	out.create<GpuMat>(Channel::Density, Format<float>(params_.camera.width, params_.camera.height));
	out.get<GpuMat>(Channel::Density).setTo(cv::Scalar(0.0f), cvstream);
	_renderChannel(out, Channel::Depth, Channel::Density, t, stream_);
}

void Triangular::_renderRight(ftl::rgbd::Frame &out, const Eigen::Matrix4d &t) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	float baseline = params_.camera.baseline;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 3) = baseline;
	Eigen::Matrix4f matrix = out.getPose().cast<float>() * transform.inverse();
	
	params_.m_viewMatrix = MatrixConversion::toCUDA(matrix.inverse());
	params_.m_viewMatrixInverse = MatrixConversion::toCUDA(matrix);
	params_.camera = out.getRightCamera();
	
	out.create<GpuMat>(Channel::Right, Format<uchar4>(params_.camera.width, params_.camera.height));
	out.get<GpuMat>(Channel::Right).setTo(background_, cvstream);
	out.createTexture<uchar4>(Channel::Right, true);

	// Need to re-dibr due to pose change
	if (mesh_) {
		_mesh(out, t, stream_);
	} else {
		_dibr(out, t, stream_);
	}
	_renderChannel(out, Channel::Left, Channel::Right, t, stream_);
}

void Triangular::_renderSecond(ftl::rgbd::Frame &out, ftl::codecs::Channel chan, const Eigen::Matrix4d &t) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	if (ftl::codecs::isFloatChannel(chan)) {
		out.create<GpuMat>(chan, Format<float>(params_.camera.width, params_.camera.height));
		out.get<GpuMat>(chan).setTo(cv::Scalar(0.0f), cvstream);
	} else {
		out.create<GpuMat>(chan, Format<uchar4>(params_.camera.width, params_.camera.height));
		out.get<GpuMat>(chan).setTo(background_, cvstream);
	}
	_renderChannel(out, chan, chan, t, stream_);
}

void Triangular::_render(ftl::rgbd::FrameSet &in, ftl::rgbd::Frame &out, Channel chan, const Eigen::Matrix4d &t) {
	const auto &camera = out.getLeftCamera();
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);
	//cudaSafeCall(cudaSetDevice(scene_->getCUDADevice()));

	_updateParameters(out);

	// Create all the required channels
	_allocateChannels(out);

	// Apply a colour background
	if (env_image_.empty() || !value("environment_enabled", false)) {
		out.get<GpuMat>(Channel::Colour).setTo(background_, cvstream);
	} else {
		auto pose = params_.m_viewMatrixInverse.getFloat3x3();
		ftl::cuda::equirectangular_reproject(
			env_tex_,
			out.createTexture<uchar4>(Channel::Colour, true),
			camera, pose, stream_);
	}

	//cudaSafeCall(cudaStreamSynchronize(stream_));

	// Render source specific debug info into colour channels
	_preprocessColours();

	//cudaSafeCall(cudaStreamSynchronize(stream_));

	if (mesh_) {
		// Render depth channel using triangles
		_mesh(out, t, stream_);
	} else {
		// Render depth channel as a point cloud
		_dibr(out, t, stream_);
	}

	//cudaSafeCall(cudaStreamSynchronize(stream_));

	// Reprojection of colours onto surface
	auto main_channel = (scene_->frames[0].hasChannel(Channel::ColourHighRes)) ? Channel::ColourHighRes : Channel::Colour;
	_renderChannel(out, main_channel, Channel::Colour, t, stream_);

	//cudaSafeCall(cudaStreamSynchronize(stream_));

	// Debug colour info relating to the rendering process
	_postprocessColours(out);

	//cudaSafeCall(cudaStreamSynchronize(stream_));

	// Support rendering of a second channel without redoing all the work
	switch(chan) {
	case Channel::None			:
	case Channel::Left			:
	case Channel::Depth			: break;
	case Channel::ColourNormals	: _renderNormals(out); break;
	case Channel::Density		: _renderDensity(out, t); break;
	case Channel::Right			: _renderRight(out, t); break;
	default						: _renderSecond(out, chan, t);
	}
}

bool Triangular::render(ftl::rgbd::FrameSet &in, ftl::rgbd::Frame &out, Channel chan, const Eigen::Matrix4d &t) {
	if (scene_) {
		LOG(WARNING) << "Already rendering...";
		return false;
	}
	scene_ = &in;
	if (scene_->frames.size() == 0) {
		scene_ = nullptr;
		return false;
	}

	bool success = true;

	try {
		_render(in, out, chan, t);
		cudaSafeCall(cudaStreamSynchronize(stream_));
	} catch (std::exception &e) {
		LOG(ERROR) << "Exception in render: " << e.what();
		success = false;
	}

	last_frame_ = scene_->timestamp;
	scene_ = nullptr;
	return success;
}
