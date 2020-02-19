#include <loguru.hpp>

#include "ftl/operators/disparity.hpp"
#include "disparity/cuda.hpp"

#ifdef HAVE_OPTFLOW

using ftl::operators::OpticalFlowTemporalSmoothing;

using ftl::codecs::Channel;
using ftl::rgbd::Frame;
using ftl::rgbd::Source;

using cv::Mat;
using cv::Size;

using std::vector;

template<typename T> static bool inline isValidDisparity(T d) { return (0.0 < d) && (d < 256.0); } // TODO

OpticalFlowTemporalSmoothing::OpticalFlowTemporalSmoothing(ftl::Configurable* cfg, const std::tuple<ftl::codecs::Channel> &params) :
		ftl::operators::Operator(cfg) {
	channel_ = std::get<0>(params);
	_init(cfg);
}

OpticalFlowTemporalSmoothing::OpticalFlowTemporalSmoothing(ftl::Configurable* cfg) :
		ftl::operators::Operator(cfg) {
	
	_init(cfg);
}

void OpticalFlowTemporalSmoothing::_init(ftl::Configurable* cfg) {
	size_ = Size(0, 0);

	n_max_ = cfg->value("history_size", 16);
	if (n_max_ < 1) {
		LOG(WARNING) << "History length must be larger than 0, using default (0)";
		n_max_ = 7;
	}

	if (n_max_ > 16) {
		// TODO: cuda kernel uses fixed size buffer
		LOG(WARNING) << "History length can't be larger than 16 (TODO)";
		n_max_ = 16;
	}

	threshold_ = cfg->value("threshold", 5.0f);

	cfg->on("threshold", [this](const ftl::config::Event&) {
		float threshold = config()->value("threshold", 5.0f);
		if (threshold < 0.0) {
			LOG(WARNING) << "invalid threshold " << threshold << ", value must be positive";
		}
		else {
			threshold_ = threshold;
			//init();
		}
	});

	cfg->on("history_size", [this, &cfg](const ftl::config::Event&) {
		int n_max = cfg->value("history_size", 7);

		if (n_max < 1) {
			LOG(WARNING) << "History length must be larger than 0";
		}
		else if (n_max_ > 16) {
			// TODO: cuda kernel uses fixed size buffer
			LOG(WARNING) << "History length can't be larger than 16 (TODO)";
		}
		else {
			n_max_ = n_max;
			init();
		}
	});
}

OpticalFlowTemporalSmoothing::~OpticalFlowTemporalSmoothing() {}

bool OpticalFlowTemporalSmoothing::init() {
	if (size_ == Size(0, 0)) { return false; }
	history_.create(cv::Size(size_.width * n_max_, size_.height), CV_32FC1);
	history_.setTo(0.0);
	return true;
}

bool OpticalFlowTemporalSmoothing::apply(Frame &in, Frame &out, cudaStream_t stream) {
	if (!out.hasChannel(channel_) || !in.hasChannel(Channel::Flow)) { return false; }

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	const cv::cuda::GpuMat &optflow = in.get<cv::cuda::GpuMat>(Channel::Flow);
	cv::cuda::GpuMat &data = out.get<cv::cuda::GpuMat>(channel_);
	
	if (data.size() != size_) {
		size_ = data.size();
		if (!init()) { return false; }
	}

	ftl::cuda::optflow_filter(data, optflow, history_, in.get<cv::cuda::GpuMat>(Channel::Support1), n_max_, threshold_, config()->value("filling", false), cvstream);
	
	return true;
}

#endif  // HAVE_OPTFLOW
