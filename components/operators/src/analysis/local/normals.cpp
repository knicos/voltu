#include <ftl/operators/normals.hpp>
#include <ftl/cuda/normals.hpp>
#include <ftl/utility/matrix_conversion.hpp>

using ftl::operators::Normals;
using ftl::operators::NormalDot;
using ftl::operators::SmoothNormals;
using ftl::codecs::Channel;
using ftl::rgbd::Format;

Normals::Normals(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

Normals::~Normals() {

}

bool Normals::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Depth)) {
		out.message(ftl::data::Message::Warning_MISSING_CHANNEL, "Missing Depth Channel in Normals operator");
		//throw FTL_Error("Missing depth channel in Normals operator");
		return false;
	}

	if (out.hasChannel(Channel::Normals)) {
		//LOG(WARNING) << "Output already has normals";
		return true;
	}

	ftl::cuda::normals(
		out.createTexture<half4>(Channel::Normals, ftl::rgbd::Format<half4>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
		in.createTexture<float>(Channel::Depth),
		in.getLeftCamera(), stream
	);

	return true;
}

// =============================================================================

NormalDot::NormalDot(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

NormalDot::~NormalDot() {

}

bool NormalDot::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (!in.hasChannel(Channel::Depth)) {
		out.message(ftl::data::Message::Warning_MISSING_CHANNEL, "Missing Depth Channel in Normals operator");
		//throw FTL_Error("Missing depth channel in Normals operator");
		return false;
	}

	if (out.hasChannel(Channel::Normals)) {
		//LOG(WARNING) << "Output already has normals";
		return true;
	}

	ftl::cuda::normals_dot(
		out.createTexture<float>(Channel::Normals, ftl::rgbd::Format<float>(in.get<cv::cuda::GpuMat>(Channel::Depth).size())),
		in.createTexture<float>(Channel::Depth),
		in.getLeftCamera(), stream
	);

	return true;
}

// =============================================================================


SmoothNormals::SmoothNormals(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

SmoothNormals::~SmoothNormals() {

}

bool SmoothNormals::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
    float smoothing = config()->value("normal_smoothing", 0.02f);
    int radius = max(0, min(config()->value("radius",1), 5));

	if (!in.hasChannel(Channel::Depth)) {
		out.message(ftl::data::Message::Warning_MISSING_CHANNEL, "Missing Depth Channel in Normals operator");
		throw FTL_Error("Missing depth channel in SmoothNormals operator");
	}

	if (out.hasChannel(Channel::Normals)) {
		//LOG(WARNING) << "Output already has normals";
		return true;
	}

	auto &depth = in.get<cv::cuda::GpuMat>(Channel::Depth);

	temp_.create(depth.size());

    ftl::cuda::normals(
		out.createTexture<half4>(Channel::Normals, Format<half4>(depth.size())),
		temp_,
		in.createTexture<float>(Channel::Depth),
		radius, smoothing,
		in.getLeftCamera(),
		MatrixConversion::toCUDA(in.getPose().cast<float>().inverse()).getFloat3x3(),
		MatrixConversion::toCUDA(in.getPose().cast<float>()).getFloat3x3(),
		stream
	);


	return true;
}
