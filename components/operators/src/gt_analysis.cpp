#include <ftl/operators/gt_analysis.hpp>
#include <ftl/operators/cuda/gt.hpp>

using ftl::operators::GTAnalysis;
using ftl::codecs::Channel;
using std::string;

GTAnalysis::GTAnalysis(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {
	cudaMalloc(&output_, sizeof(ftl::cuda::GTAnalysisData));
}

GTAnalysis::~GTAnalysis() {
	cudaFree(output_);
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}

struct Options {
	float t_min;
	float t_max;
	uchar4 colour;
};

static const std::vector<Options> options_disparity = {
		{-INFINITY, INFINITY, {0,0,224,255}},
		{-INFINITY, 2.0, {66,174,255,255}},
		{-INFINITY, 1.0, {16,192,16,255}},
		{-INFINITY, 0.25, {64,255,64,255}},
};

static const std::vector<Options> options_depth = {
		{-INFINITY, INFINITY, {0,0,224,255}},
		{-INFINITY, 0.1, {66,174,255,255}},
		{-INFINITY, 0.025, {16,192,16,255}},
		{-INFINITY, 0.01, {64,255,64,255}},
};

static void report(std::vector<std::string> &msgs, const ftl::cuda::GTAnalysisData &data,
	const Options &o, float npixels, const std::string &unit="", float scale=1.0f) {

	msgs.push_back(	"(" + to_string_with_precision(o.t_min, 2)
					+ "," + to_string_with_precision(o.t_max, 2) + "] ");
	msgs.push_back("valid: " + to_string_with_precision(100.0f*data.good/data.valid, 1) + "%, "
					+ "all: " + to_string_with_precision(100.0f*data.good/npixels, 1) + "%");
	msgs.push_back(	"RMS: "
					+ to_string_with_precision(sqrt(data.err_sq/data.good) * scale, 2)
					+ (unit.empty() ? "" : " " + unit));
}

bool GTAnalysis::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {

	if (!in.hasChannel(Channel::Depth) || !in.hasChannel(Channel::GroundTruth)) {
		return true;
	}

	std::vector<std::string> msgs;
	if (in.hasChannel(Channel::Messages)) { msgs = in.get<std::vector<std::string>>(Channel::Messages); }

	bool use_disp = config()->value("use_disparity", true);
	auto &dmat = in.get<cv::cuda::GpuMat>(Channel::Depth);
	const float npixels = dmat.rows * dmat.cols;
	ftl::cuda::GTAnalysisData err;

	for (const auto &o : (use_disp ? options_disparity : options_depth)) {
		if (config()->value("show_colour", false)) {
			ftl::cuda::gt_analysis(
				in.createTexture<uchar4>(Channel::Colour),
				in.createTexture<float>(Channel::Depth),
				in.createTexture<float>(Channel::GroundTruth),
				in.createTexture<uchar>(Channel::Mask),
				output_,
				in.getLeft(),
				o.t_min,
				o.t_max,
				o.colour,
				use_disp,
				stream
			);
		}
		else {
			ftl::cuda::gt_analysis(
				in.createTexture<float>(Channel::Depth),
				in.createTexture<float>(Channel::GroundTruth),
				in.createTexture<uchar>(Channel::Mask),
				output_,
				in.getLeft(),
				o.t_min,
				o.t_max,
				use_disp,
				stream
			);
		}

		cudaMemcpy(&err, output_, sizeof(err), cudaMemcpyDeviceToHost);
		msgs.push_back(" ");
		if (use_disp) 	{ report(msgs, err, o, npixels, "px", 1.0); }
		else 			{ report(msgs, err, o, npixels, "mm", 1000.0); }
	}

	in.create<std::vector<std::string>>(Channel::Messages) = msgs;

	return true;
}
