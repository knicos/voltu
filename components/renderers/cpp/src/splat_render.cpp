#include <ftl/render/splat_render.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include "splatter_cuda.hpp"
#include <ftl/cuda/points.hpp>
#include <ftl/cuda/normals.hpp>

#include <opencv2/core/cuda_stream_accessor.hpp>

#include <string>

using ftl::render::Splatter;
using ftl::rgbd::Channel;
using ftl::rgbd::Channels;
using ftl::rgbd::Format;
using cv::cuda::GpuMat;
using std::stoul;

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

Splatter::Splatter(nlohmann::json &config, ftl::rgbd::FrameSet *fs) : ftl::render::Renderer(config), scene_(fs) {
	if (config["clipping"].is_object()) {
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
	} else {
		clipping_ = false;
	}

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

	splat_ = value("splatting", true);
	on("splatting", [this](const ftl::config::Event &e) {
		splat_ = value("splatting", true);
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
}

Splatter::~Splatter() {

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

template <typename T>
void Splatter::__blendChannel(ftl::rgbd::Frame &output, ftl::rgbd::Channel in, ftl::rgbd::Channel out, cudaStream_t stream) {
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
}

void Splatter::_blendChannel(ftl::rgbd::Frame &output, ftl::rgbd::Channel in, ftl::rgbd::Channel out, cudaStream_t stream) {
	int type = output.get<GpuMat>(out).type(); // == CV_32F; //ftl::rgbd::isFloatChannel(channel);
	
	switch (type) {
	case CV_32F		: __blendChannel<float>(output, in, out, stream); break;
	case CV_32FC4	: __blendChannel<float4>(output, in, out, stream); break;
	case CV_8UC4	: __blendChannel<uchar4>(output, in, out, stream); break;
	default			: LOG(ERROR) << "Invalid output channel format";
	}
}

void Splatter::_dibr(cudaStream_t stream) {
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	temp_.get<GpuMat>(Channel::Depth2).setTo(cv::Scalar(0x7FFFFFFF), cvstream);

	for (size_t i=0; i < scene_->frames.size(); ++i) {
		auto &f = scene_->frames[i];
		auto *s = scene_->sources[i];

		if (f.empty(Channel::Depth + Channel::Colour)) {
			LOG(ERROR) << "Missing required channel";
			continue;
		}

		ftl::cuda::dibr_merge(
			f.createTexture<float4>(Channel::Points),
			f.createTexture<float4>(Channel::Normals),
			temp_.createTexture<int>(Channel::Depth2),
			params_, backcull_, stream
		);

		//LOG(INFO) << "DIBR DONE";
	}
}

void Splatter::_renderChannel(
		ftl::rgbd::Frame &out,
		Channel channel_in, Channel channel_out, cudaStream_t stream)
{
	if (channel_out == Channel::None || channel_in == Channel::None) return;
	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	if (scene_->frames.size() < 1) return;
	bool is_float = out.get<GpuMat>(channel_out).type() == CV_32F; //ftl::rgbd::isFloatChannel(channel);
	bool is_4chan = out.get<GpuMat>(channel_out).type() == CV_32FC4;


	temp_.createTexture<float4>(Channel::Colour);
	temp_.createTexture<float>(Channel::Contribution);

	// Generate initial normals for the splats
	accum_.create<GpuMat>(Channel::Normals, Format<float4>(params_.camera.width, params_.camera.height));
	_blendChannel(accum_, Channel::Normals, Channel::Normals, stream);

	// Estimate point density
	accum_.create<GpuMat>(Channel::Density, Format<float>(params_.camera.width, params_.camera.height));
	_blendChannel(accum_, Channel::Depth, Channel::Density, stream);

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

	//if (splat_) {
		_blendChannel(accum_, channel_in, channel_out, stream);
	//} else {
	//	_blendChannel(out, channel, channel, stream);
	//}

	// Now splat the points
	if (splat_) {
		if (is_4chan) {
			ftl::cuda::splat(
				accum_.getTexture<float4>(Channel::Normals),
				accum_.getTexture<float>(Channel::Density),
				accum_.getTexture<float4>(channel_out),
				temp_.getTexture<int>(Channel::Depth2),
				out.createTexture<float>(Channel::Depth),
				out.createTexture<float4>(channel_out),
				params_, stream
			);
		} else if (is_float) {
			ftl::cuda::splat(
				accum_.getTexture<float4>(Channel::Normals),
				accum_.getTexture<float>(Channel::Density),
				accum_.getTexture<float>(channel_out),
				temp_.getTexture<int>(Channel::Depth2),
				out.createTexture<float>(Channel::Depth),
				out.createTexture<float>(channel_out),
				params_, stream
			);
		} else {
			ftl::cuda::splat(
				accum_.getTexture<float4>(Channel::Normals),
				accum_.getTexture<float>(Channel::Density),
				accum_.getTexture<uchar4>(channel_out),
				temp_.getTexture<int>(Channel::Depth2),
				out.createTexture<float>(Channel::Depth),
				out.createTexture<uchar4>(channel_out),
				params_, stream
			);
		}
	} else {
		// Swap accum frames directly to output.
	}
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
	temp_.create<GpuMat>(Channel::Contribution, Format<float>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth2, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Normals, Format<float4>(camera.width, camera.height));

	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	// Parameters object to pass to CUDA describing the camera
	SplatParams &params = params_;
	params.m_flags = 0;
	if (value("show_discontinuity_mask", false)) params.m_flags |= ftl::render::kShowDisconMask;
	if (value("normal_weight_colours", true)) params.m_flags |= ftl::render::kNormalWeightColours;
	params.m_viewMatrix = MatrixConversion::toCUDA(src->getPose().cast<float>().inverse());
	params.m_viewMatrixInverse = MatrixConversion::toCUDA(src->getPose().cast<float>());
	params.camera = camera;

	// Clear all channels to 0 or max depth

	out.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(1000.0f), cvstream);
	out.get<GpuMat>(Channel::Colour).setTo(background_, cvstream);

	//LOG(INFO) << "Render ready: " << camera.width << "," << camera.height;

	temp_.createTexture<int>(Channel::Depth);
	//temp_.get<GpuMat>(Channel::Normals).setTo(cv::Scalar(0.0f,0.0f,0.0f,0.0f), cvstream);

	// First make sure each input has normals
	temp_.createTexture<float4>(Channel::Normals);
	for (int i=0; i<scene_->frames.size(); ++i) {
		auto &f = scene_->frames[i];
		auto s = scene_->sources[i];

		// Needs to create points channel first?
		if (!f.hasChannel(Channel::Points)) {
			//LOG(INFO) << "Creating points... " << s->parameters().width;
			
			auto &t = f.createTexture<float4>(Channel::Points, Format<float4>(f.get<GpuMat>(Channel::Colour).size()));
			auto pose = MatrixConversion::toCUDA(s->getPose().cast<float>()); //.inverse());
			ftl::cuda::point_cloud(t, f.createTexture<float>(Channel::Depth), s->parameters(), pose, 0, stream);

			//LOG(INFO) << "POINTS Added";
		}

		// Clip first?
		if (clipping_) {
			ftl::cuda::clipping(f.createTexture<float4>(Channel::Points), clip_, stream);
		}

		if (!f.hasChannel(Channel::Normals)) {
			Eigen::Matrix4f matrix =  s->getPose().cast<float>();
			auto pose = MatrixConversion::toCUDA(matrix);

			auto &g = f.get<GpuMat>(Channel::Colour);
			ftl::cuda::normals(f.createTexture<float4>(Channel::Normals, Format<float4>(g.cols, g.rows)),
				temp_.getTexture<float4>(Channel::Normals),  // FIXME: Uses assumption of vcam res same as input res
				f.getTexture<float4>(Channel::Points),
				3, 0.04f,
				s->parameters(), pose.getFloat3x3(), stream);

			if (norm_filter_ > -0.1f) {
				ftl::cuda::normal_filter(f.getTexture<float4>(Channel::Normals), f.getTexture<float4>(Channel::Points), s->parameters(), pose, norm_filter_, stream);
			}
		}
	}

	_dibr(stream);
	_renderChannel(out, Channel::Colour, Channel::Colour, stream);
	
	Channel chan = src->getChannel();
	if (chan == Channel::Depth)
	{
		//temp_.get<GpuMat>(Channel::Depth).convertTo(out.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 1000.0f, cvstream);
	} else if (chan == Channel::Normals) {
		out.create<GpuMat>(Channel::Normals, Format<float4>(camera.width, camera.height));

		// Render normal attribute
		_renderChannel(out, Channel::Normals, Channel::Normals, stream);

		// Convert normal to single float value
		temp_.create<GpuMat>(Channel::Colour, Format<uchar4>(camera.width, camera.height));
		ftl::cuda::normal_visualise(out.getTexture<float4>(Channel::Normals), temp_.createTexture<uchar4>(Channel::Colour),
				make_float3(0.3f, 0.2f, 1.0f),
				light_diffuse_,
				light_ambient_, stream);

		// Put in output as single float
		cv::cuda::swap(temp_.get<GpuMat>(Channel::Colour), out.create<GpuMat>(Channel::Normals));
		out.resetTexture(Channel::Normals);
	}
	//else if (chan == Channel::Contribution)
	//{
	//	cv::cuda::swap(temp_.get<GpuMat>(Channel::Contribution), out.create<GpuMat>(Channel::Contribution));
	//}
	else if (chan == Channel::Density) {
		out.create<GpuMat>(chan, Format<float>(camera.width, camera.height));
		out.get<GpuMat>(chan).setTo(cv::Scalar(0.0f), cvstream);
		_renderChannel(out, Channel::Depth, Channel::Density, stream);
	}
	else if (chan == Channel::Right)
	{
		Eigen::Affine3f transform(Eigen::Translation3f(camera.baseline,0.0f,0.0f));
		Eigen::Matrix4f matrix =  src->getPose().cast<float>() * transform.matrix();
		params.m_viewMatrix = MatrixConversion::toCUDA(matrix.inverse());
		params.m_viewMatrixInverse = MatrixConversion::toCUDA(matrix);
		
		out.create<GpuMat>(Channel::Right, Format<uchar4>(camera.width, camera.height));
		out.get<GpuMat>(Channel::Right).setTo(background_, cvstream);

		_dibr(stream); // Need to re-dibr due to pose change
		_renderChannel(out, Channel::Right, Channel::Right, stream);
	} else if (chan != Channel::None) {
		if (ftl::rgbd::isFloatChannel(chan)) {
			out.create<GpuMat>(chan, Format<float>(camera.width, camera.height));
			out.get<GpuMat>(chan).setTo(cv::Scalar(0.0f), cvstream);
		} else {
			out.create<GpuMat>(chan, Format<uchar4>(camera.width, camera.height));
			out.get<GpuMat>(chan).setTo(background_, cvstream);
		}
		_renderChannel(out, chan, chan, stream);
	}

	return true;
}

//void Splatter::setOutputDevice(int device) {
//	device_ = device;
//}
