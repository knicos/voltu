#include <ftl/render/colouriser.hpp>
#include "splatter_cuda.hpp"
#include <ftl/cuda/colour_cuda.hpp>
#include <ftl/cuda/normals.hpp>

#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <string>

using ftl::render::Colouriser;
using ftl::cuda::TextureObject;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;
using std::stoul;

static cv::cuda::GpuMat depth_lut;

#include "turbo_map.hpp"

/*
 * Parse a CSS style colour string into a scalar.
 */
cv::Scalar ftl::render::parseCVColour(const std::string &colour) {
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
uchar4 ftl::render::parseCUDAColour(const std::string &colour) {
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


Colouriser::Colouriser(nlohmann::json &config) : ftl::Configurable(config) {

}

Colouriser::~Colouriser() {

}

TextureObject<uchar4> &Colouriser::colourise(ftl::rgbd::Frame &f, Channel c, cudaStream_t stream) {
	const auto &vf = f.get<ftl::rgbd::VideoFrame>(c);
	if (!vf.isGPU()) {
		f.upload(c);
	}

	switch (c) {
	case Channel::Overlay		: return f.createTexture<uchar4>(c);
	case Channel::ColourHighRes	:
	case Channel::RightHighRes	:
	case Channel::Colour		:
	case Channel::Colour2		: return _processColour(f,c,stream);
	case Channel::GroundTruth	:
	case Channel::Depth			:
	case Channel::Depth2		: return _processFloat(f,c, value("depth_min", f.getLeft().minDepth), value("depth_max", f.getLeft().maxDepth), stream);
	case Channel::Normals		:
	case Channel::Normals2		: return _processNormals(f, c, stream);
	case Channel::Weights		: return _processSingle<short>(f, c, stream);
	default: throw FTL_Error("Cannot colourise channel " << (int)c);
	}
}

TextureObject<uchar4> &Colouriser::_processNormals(ftl::rgbd::Frame &f, Channel c, cudaStream_t stream) {
	cv::Size size = f.get<cv::cuda::GpuMat>(c).size();
	auto &buf = _getBuffer(size.width, size.height);

	auto light_diffuse = parseCUDAColour(value("diffuse", std::string("#e0e0e0")));
	auto light_ambient = parseCUDAColour(value("ambient", std::string("#0e0e0e")));

	light_diffuse.w = value("alpha", 0.5f)*255.0f;
	light_ambient.w = light_diffuse.w;

	auto light_pos = make_float3(value("light_x", 0.3f), value("light_y", 0.2f), value("light_z", 1.0f));

	ftl::cuda::normal_visualise(f.createTexture<half4>(c), buf,
			light_pos,
			light_diffuse,
			light_ambient, stream);
	return buf;
}

template <typename T>
TextureObject<uchar4> &Colouriser::_processSingle(ftl::rgbd::Frame &f, Channel c, cudaStream_t stream) {
	cv::Size size = f.get<cv::cuda::GpuMat>(c).size();
	auto &buf = _getBuffer(size.width, size.height);

	if (depth_lut.empty()) {
		cv::Mat lut(1,256, CV_8UC3, turbo_map);
		depth_lut.upload(lut);
	}

	float alpha = value("alpha",0.5f)*255.0f;

	ftl::cuda::lut(f.createTexture<T>(c), buf, depth_lut, 0, std::numeric_limits<T>::max(), alpha, true, stream);
	return buf;
}

TextureObject<uchar4> &Colouriser::_processFloat(ftl::rgbd::Frame &f, Channel c, float minval, float maxval, cudaStream_t stream) {
	cv::Size size = f.get<cv::cuda::GpuMat>(c).size();
	auto &buf = _getBuffer(size.width, size.height);

	if (depth_lut.empty()) {
		cv::Mat lut(1,256, CV_8UC3, turbo_map);
		depth_lut.upload(lut);
	}

	float alpha = value("alpha",0.5f)*255.0f;

	ftl::cuda::lut(f.createTexture<float>(c), buf, depth_lut, minval, maxval, alpha, false, stream);
	return buf;
}

TextureObject<uchar4> &Colouriser::_processColour(ftl::rgbd::Frame &f, Channel c, cudaStream_t stream) {
	uint8_t show_mask = value("show_mask", 0);
	bool colour_sources = value("colour_sources", false);

	if (!colour_sources && show_mask == 0) {
		return f.createTexture<uchar4>(c, true);
	}

	cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	cv::Size size = f.get<cv::cuda::GpuMat>(c).size();
	auto &buf = _getBuffer(size.width, size.height);

	if (colour_sources) {
		auto colour = HSVtoRGB(360 / 8 * f.source(), 0.6, 0.85);
		buf.to_gpumat().setTo(colour, cvstream);
	}

	if (f.hasChannel(Channel::Mask)) {
		if (show_mask > 0) {
			f.get<cv::cuda::GpuMat>(c).copyTo(buf.to_gpumat());
			ftl::cuda::show_mask(buf, f.getTexture<uint8_t>(Channel::Mask), show_mask, make_uchar4(255,0,255,255), stream);
		}
	}

	return buf;
}

TextureObject<uchar4> &Colouriser::_getBuffer(size_t width, size_t height) {
	for (auto *b : textures_) {
		if (b->width() == width && b->height() == height) return *b;
	}
	auto *nb = new ftl::cuda::TextureObject<uchar4>(width, height, true);
	textures_.push_back(nb);
	return *nb;
}
