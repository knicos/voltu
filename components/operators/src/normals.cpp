#include <ftl/operators/normals.hpp>
#include <ftl/cuda/normals.hpp>
#include <ftl/utility/matrix_conversion.hpp>

using ftl::operators::Normals;
using ftl::operators::SmoothNormals;
using ftl::codecs::Channel;
using ftl::rgbd::Format;

Normals::Normals(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

Normals::~Normals() {

}

bool Normals::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Depth)) {
		LOG(ERROR) << "Missing depth channel in Normals operator";
		return false;
	}

	if (out.hasChannel(Channel::Normals)) {
		LOG(WARNING) << "Output already has normals";
	}

	ftl::cuda::normals(
		out.createTexture<float4>(Channel::Normals, ftl::rgbd::Format<float4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
		in.createTexture<float>(Channel::Depth),
		s->parameters(), 0
	);

	return true;
}


SmoothNormals::SmoothNormals(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

SmoothNormals::~SmoothNormals() {

}

bool SmoothNormals::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
    float smoothing = config()->value("normal_smoothing", 0.02f);
    int radius = max(0, min(config()->value("radius",1), 5));

	if (!in.hasChannel(Channel::Depth)) {
		LOG(ERROR) << "Missing depth channel in SmoothNormals operator";
		return false;
	}

	if (out.hasChannel(Channel::Normals)) {
		LOG(WARNING) << "Output already has normals";
	}

	auto &depth = in.get<cv::cuda::GpuMat>(Channel::Depth);

	temp_.create(depth.size());

    ftl::cuda::normals(
		out.createTexture<float4>(Channel::Normals, Format<float4>(depth.size())),
		temp_,
		in.createTexture<float>(Channel::Depth),
		radius, smoothing,
		s->parameters(),
		MatrixConversion::toCUDA(s->getPose().cast<float>().inverse()).getFloat3x3(),
		MatrixConversion::toCUDA(s->getPose().cast<float>()).getFloat3x3(),
		stream
	);


	return true;
}