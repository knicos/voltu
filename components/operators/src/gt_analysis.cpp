#include <ftl/operators/gt_analysis.hpp>
#include <ftl/operators/gt_cuda.hpp>

using ftl::operators::GTAnalysis;
using ftl::codecs::Channel;
using std::string;

GTAnalysis::GTAnalysis(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {
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

bool GTAnalysis::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (in.hasChannel(Channel::Depth) && in.hasChannel(Channel::GroundTruth)) {
		if (config()->value("show_colour", false)) {
			ftl::cuda::gt_analysis(
				in.createTexture<uchar4>(Channel::Colour),
				in.createTexture<float>(Channel::Depth),
				in.createTexture<float>(Channel::GroundTruth),
				output_,
				in.getLeft(),
				config()->value("bad_threshold", 2.0f),
				config()->value("viz_threshold", 5.0f),
				stream
			);
		} else {
			ftl::cuda::gt_analysis(
				in.createTexture<float>(Channel::Depth),
				in.createTexture<float>(Channel::GroundTruth),
				output_,
				in.getLeft(),
				config()->value("bad_threshold", 2.0f),
				stream
			);
		}

		ftl::cuda::GTAnalysisData anal;
		cudaMemcpy(&anal, output_, sizeof(anal), cudaMemcpyDeviceToHost);

		auto &dmat = in.get<cv::cuda::GpuMat>(Channel::Depth);
		int totalvalid = dmat.cols*dmat.rows - anal.invalid - anal.masked;
		//int totaltested = dmat.cols*dmat.rows - anal.masked;

		float pbad = float(anal.bad) / float(totalvalid) * 100.0f;
		float pinvalid = float(anal.invalid) / float(dmat.cols*dmat.rows - anal.masked) * 100.0f;
		float avgerr = anal.totalerror / float(totalvalid) * 100.0f;

		std::vector<std::string> msgs;
		if (in.hasChannel(Channel::Messages)) in.get(Channel::Messages, msgs);

		msgs.push_back(string("Bad %: ") + to_string_with_precision(pbad, 1));
		msgs.push_back(string("Invalid %: ") + to_string_with_precision(pinvalid,1));
		msgs.push_back(string("Avg Error: ") + to_string_with_precision(avgerr, 2));

		in.create(Channel::Messages, msgs);
	}

	return true;
}
