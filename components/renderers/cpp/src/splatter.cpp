#include <ftl/render/splat_render.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include "splatter_cuda.hpp"
#include <ftl/cuda/points.hpp>
#include <ftl/cuda/normals.hpp>
#include <ftl/cuda/mask.hpp>

#include <opencv2/core/cuda_stream_accessor.hpp>

#include <string>

using ftl::render::Splatter;
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

	light_pos_ = make_float3(value("light_x", 0.3f), value("light_y", 0.2f), value("light_z", 1.0f));
	on("light_x", [this](const ftl::config::Event &e) { light_pos_.x = value("light_x", 0.3f); });
	on("light_y", [this](const ftl::config::Event &e) { light_pos_.y = value("light_y", 0.3f); });
	on("light_z", [this](const ftl::config::Event &e) { light_pos_.z = value("light_z", 0.3f); });

	cudaSafeCall(cudaStreamCreate(&stream_));
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
void Splatter::__blendChannel(ftl::rgbd::Frame &output, ftl::codecs::Channel in, ftl::codecs::Channel out, cudaStream_t stream) {
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

void Splatter::_blendChannel(ftl::rgbd::Frame &output, ftl::codecs::Channel in, ftl::codecs::Channel out, cudaStream_t stream) {
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
	// Put normals in camera space here...
	ftl::cuda::transform_normals(accum_.getTexture<float4>(Channel::Normals), params_.m_viewMatrix.getFloat3x3(), stream);

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
		accum_.swapTo(Channels(channel_out), out);
	}
}

bool Splatter::render(ftl::rgbd::VirtualSource *src, ftl::rgbd::Frame &out, const Eigen::Matrix4d &t) {
	SHARED_LOCK(scene_->mtx, lk);
	if (!src->isReady()) return false;

	scene_->upload(Channel::Colour + Channel::Depth, stream_);

	const auto &camera = src->parameters();
	//cudaSafeCall(cudaSetDevice(scene_->getCUDADevice()));

	// Create all the required channels
	
	out.create<GpuMat>(Channel::Depth, Format<float>(camera.width, camera.height));
	out.create<GpuMat>(Channel::Colour, Format<uchar4>(camera.width, camera.height));


	if (scene_->frames.size() == 0) return false;
	auto &g = scene_->frames[0].get<GpuMat>(Channel::Colour);

	temp_.create<GpuMat>(Channel::Colour, Format<float4>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Contribution, Format<float>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Depth2, Format<int>(camera.width, camera.height));
	temp_.create<GpuMat>(Channel::Normals, Format<float4>(g.cols, g.rows));

	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	// Parameters object to pass to CUDA describing the camera
	SplatParams &params = params_;
	params.m_flags = 0;
	//if () params.m_flags |= ftl::render::kShowDisconMask;
	if (value("normal_weight_colours", true)) params.m_flags |= ftl::render::kNormalWeightColours;
	params.m_viewMatrix = MatrixConversion::toCUDA(src->getPose().cast<float>().inverse());
	params.m_viewMatrixInverse = MatrixConversion::toCUDA(src->getPose().cast<float>());
	params.camera = camera;
	// Clear all channels to 0 or max depth

	out.get<GpuMat>(Channel::Depth).setTo(cv::Scalar(1000.0f), cvstream);
	out.get<GpuMat>(Channel::Colour).setTo(background_, cvstream);

	//LOG(INFO) << "Render ready: " << camera.width << "," << camera.height;

	bool show_discon = value("show_discontinuity_mask", false);
	bool show_fill = value("show_filled", false);

	temp_.createTexture<int>(Channel::Depth);
	//temp_.get<GpuMat>(Channel::Normals).setTo(cv::Scalar(0.0f,0.0f,0.0f,0.0f), cvstream);

	// First make sure each input has normals
	temp_.createTexture<float4>(Channel::Normals);
	for (int i=0; i<scene_->frames.size(); ++i) {
		auto &f = scene_->frames[i];
		auto s = scene_->sources[i];

		if (f.hasChannel(Channel::Mask)) {
			if (show_discon) {
				ftl::cuda::show_mask(f.getTexture<uchar4>(Channel::Colour), f.getTexture<int>(Channel::Mask), Mask::kMask_Discontinuity, make_uchar4(0,0,255,255), stream_);
			}
			if (show_fill) {
				ftl::cuda::show_mask(f.getTexture<uchar4>(Channel::Colour), f.getTexture<int>(Channel::Mask), Mask::kMask_Filled, make_uchar4(0,255,0,255), stream_);
			}
		}

		// Needs to create points channel first?
		if (!f.hasChannel(Channel::Points)) {
			//LOG(INFO) << "Creating points... " << s->parameters().width;
			
			auto &t = f.createTexture<float4>(Channel::Points, Format<float4>(f.get<GpuMat>(Channel::Colour).size()));
			auto pose = MatrixConversion::toCUDA(s->getPose().cast<float>()); //.inverse());
			ftl::cuda::point_cloud(t, f.createTexture<float>(Channel::Depth), s->parameters(), pose, 0, stream_);

			//LOG(INFO) << "POINTS Added";
		}

		// Clip first?
		if (clipping_) {
			ftl::cuda::clipping(f.createTexture<float4>(Channel::Points), clip_, stream_);
		}

		if (!f.hasChannel(Channel::Normals)) {
			Eigen::Matrix4f matrix =  s->getPose().cast<float>().transpose();
			auto pose = MatrixConversion::toCUDA(matrix);

			auto &g = f.get<GpuMat>(Channel::Colour);
			ftl::cuda::normals(f.createTexture<float4>(Channel::Normals, Format<float4>(g.cols, g.rows)),
				temp_.getTexture<float4>(Channel::Normals),
				f.getTexture<float4>(Channel::Points),
				1, 0.02f,
				s->parameters(), pose.getFloat3x3(), stream_);

			if (norm_filter_ > -0.1f) {
				ftl::cuda::normal_filter(f.getTexture<float4>(Channel::Normals), f.getTexture<float4>(Channel::Points), s->parameters(), pose, norm_filter_, stream_);
			}
		}
	}

	Channel chan = src->getChannel();

	int aligned_source = value("aligned_source",-1);
	if (aligned_source >= 0 && aligned_source < scene_->frames.size()) {
		// FIXME: Output may not be same resolution as source!
		cudaSafeCall(cudaStreamSynchronize(stream_));
		scene_->frames[aligned_source].copyTo(Channel::Depth + Channel::Colour, out);

		if (chan == Channel::Normals) {
			// Convert normal to single float value
			temp_.create<GpuMat>(Channel::Colour, Format<uchar4>(camera.width, camera.height)).setTo(cv::Scalar(0,0,0,0), cvstream);
			ftl::cuda::normal_visualise(scene_->frames[aligned_source].getTexture<float4>(Channel::Normals), temp_.createTexture<uchar4>(Channel::Colour),
					light_pos_,
					light_diffuse_,
					light_ambient_, stream_);

			// Put in output as single float
			cv::cuda::swap(temp_.get<GpuMat>(Channel::Colour), out.create<GpuMat>(Channel::Normals));
			out.resetTexture(Channel::Normals);
		}

		return true;
	}

	_dibr(stream_);
	_renderChannel(out, Channel::Colour, Channel::Colour, stream_);
	
	if (chan == Channel::Depth)
	{
		//temp_.get<GpuMat>(Channel::Depth).convertTo(out.get<GpuMat>(Channel::Depth), CV_32F, 1.0f / 1000.0f, cvstream);
	} else if (chan == Channel::Normals) {
		out.create<GpuMat>(Channel::Normals, Format<float4>(camera.width, camera.height));

		// Render normal attribute
		_renderChannel(out, Channel::Normals, Channel::Normals, stream_);

		// Convert normal to single float value
		temp_.create<GpuMat>(Channel::Colour, Format<uchar4>(camera.width, camera.height)).setTo(cv::Scalar(0,0,0,0), cvstream);
		ftl::cuda::normal_visualise(out.getTexture<float4>(Channel::Normals), temp_.createTexture<uchar4>(Channel::Colour),
				light_pos_,
				light_diffuse_,
				light_ambient_, stream_);

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
		_renderChannel(out, Channel::Depth, Channel::Density, stream_);
	}
	else if (chan == Channel::Right)
	{
		float baseline = camera.baseline;
		
		//Eigen::Translation3f translation(baseline, 0.0f, 0.0f);
		//Eigen::Affine3f transform(translation);
		//Eigen::Matrix4f matrix = transform.matrix() * src->getPose().cast<float>();

		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
		transform(0, 3) = baseline;
		Eigen::Matrix4f matrix = transform.inverse() * src->getPose().cast<float>();
		
		params.m_viewMatrix = MatrixConversion::toCUDA(matrix.inverse());
		params.m_viewMatrixInverse = MatrixConversion::toCUDA(matrix);

		params.camera = src->parameters(Channel::Right);
		
		out.create<GpuMat>(Channel::Right, Format<uchar4>(camera.width, camera.height));
		out.get<GpuMat>(Channel::Right).setTo(background_, cvstream);

		_dibr(stream_); // Need to re-dibr due to pose change
		_renderChannel(out, Channel::Left, Channel::Right, stream_);

	} else if (chan != Channel::None) {
		if (ftl::codecs::isFloatChannel(chan)) {
			out.create<GpuMat>(chan, Format<float>(camera.width, camera.height));
			out.get<GpuMat>(chan).setTo(cv::Scalar(0.0f), cvstream);
		} else {
			out.create<GpuMat>(chan, Format<uchar4>(camera.width, camera.height));
			out.get<GpuMat>(chan).setTo(background_, cvstream);
		}
		_renderChannel(out, chan, chan, stream_);
	}

	cudaSafeCall(cudaStreamSynchronize(stream_));
	return true;
}

//void Splatter::setOutputDevice(int device) {
//	device_ = device;
//}
