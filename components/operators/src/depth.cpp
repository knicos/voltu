#include <ftl/operators/disparity.hpp>

#include "ftl/operators/smoothing.hpp"
#include "ftl/operators/colours.hpp"
#include "ftl/operators/normals.hpp"
#include "ftl/operators/filling.hpp"
#include "ftl/operators/segmentation.hpp"
#include "ftl/operators/disparity.hpp"
#include "ftl/operators/depth.hpp"
#include "ftl/operators/mask.hpp"
#include "ftl/operators/opticalflow.hpp"

#include "./disparity/opencv/disparity_bilateral_filter.hpp"

using ftl::operators::DepthChannel;
using ftl::operators::DepthBilateralFilter;
using ftl::codecs::Channel;
using cv::Mat;
using cv::cuda::GpuMat;

static void calc_color_weighted_table(GpuMat& table_color, float sigma_range, int len)
{
	Mat cpu_table_color(1, len, CV_32F);

	float* line = cpu_table_color.ptr<float>();

	for(int i = 0; i < len; i++)
		line[i] = static_cast<float>(std::exp(-double(i * i) / (2 * sigma_range * sigma_range)));

	table_color.upload(cpu_table_color);
}

static void calc_space_weighted_filter(GpuMat& table_space, int win_size, float dist_space)
{
	int half = (win_size >> 1);

	Mat cpu_table_space(half + 1, half + 1, CV_32F);

	for (int y = 0; y <= half; ++y)
	{
		float* row = cpu_table_space.ptr<float>(y);
		for (int x = 0; x <= half; ++x)
			row[x] = exp(-sqrt(float(y * y) + float(x * x)) / dist_space);
	}

	table_space.upload(cpu_table_space);
}

// ==== Depth Bilateral Filter =================================================

DepthBilateralFilter::DepthBilateralFilter(ftl::Configurable* cfg) :
		ftl::operators::Operator(cfg) {

	scale_ = 16.0;
	radius_ = cfg->value("radius", 7);
	iter_ = cfg->value("iter", 2);
	sigma_range_ = 10.0f;
	edge_disc_ = cfg->value("edge_discontinuity", 0.04f);
	max_disc_ = cfg->value("max_discontinuity", 0.1f);
	channel_ = Channel::Depth;

	cfg->on("edge_discontinuity", [this](const ftl::config::Event &e) {
		edge_disc_ = config()->value("edge_discontinuity", 0.04f);
	});
	cfg->on("max_discontinuity", [this](const ftl::config::Event &e) {
		max_disc_ = config()->value("max_discontinuity", 0.1f);
	});


	calc_color_weighted_table(table_color_, sigma_range_, 255);
    calc_space_weighted_filter(table_space_, radius_ * 2 + 1, radius_ + 1.0f);
}

DepthBilateralFilter::DepthBilateralFilter(ftl::Configurable* cfg, const std::tuple<ftl::codecs::Channel> &p) :
		ftl::operators::Operator(cfg) {

	scale_ = 16.0;
	radius_ = cfg->value("radius", 7);
	iter_ = cfg->value("iter", 2);
	sigma_range_ = 10.0f;
	edge_disc_ = cfg->value("edge_discontinuity", 0.04f);
	max_disc_ = cfg->value("max_discontinuity", 0.1f);
	channel_ = std::get<0>(p);

	cfg->on("edge_discontinuity", [this](const ftl::config::Event &e) {
		edge_disc_ = config()->value("edge_discontinuity", 0.04f);
	});
	cfg->on("max_discontinuity", [this](const ftl::config::Event &e) {
		max_disc_ = config()->value("max_discontinuity", 0.1f);
	});


	calc_color_weighted_table(table_color_, sigma_range_, 255);
    calc_space_weighted_filter(table_space_, radius_ * 2 + 1, radius_ + 1.0f);
}

bool DepthBilateralFilter::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out,
									 cudaStream_t stream) {

	if (!in.hasChannel(Channel::Colour)) {
		throw FTL_Error("Joint Bilateral Filter is missing Colour");
	} else if (!in.hasChannel(Channel::Depth)) {
		throw FTL_Error("Joint Bilateral Filter is missing Depth");
	}

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
	const GpuMat &rgb = in.get<GpuMat>(Channel::Colour);
	GpuMat &depth = in.get<GpuMat>(channel_);

	UNUSED(rgb);
	UNUSED(depth);

	// FIXME: Not working right now
	//ftl::cuda::device::disp_bilateral_filter::disp_bilateral_filter<float>(depth, rgb, rgb.channels(), iter_,
	//		table_color_.ptr<float>(), (float *)table_space_.data, table_space_.step / sizeof(float),
	//		radius_, edge_disc_, max_disc_, stream);

	//disp_in.convertTo(disp_int_, CV_16SC1, scale_, cvstream);
	//filter_->apply(disp_in, rgb, disp_out, cvstream);
	//disp_int_result_.convertTo(disp_out, disp_in.type(), 1.0/scale_, cvstream);
	return true;
}

// =============================================================================

DepthChannel::DepthChannel(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {
	pipe_ = nullptr;
}

DepthChannel::~DepthChannel() {

}

void DepthChannel::_createPipeline(size_t size) {
	if (pipe_ != nullptr) return;

	pipe_ = ftl::config::create<ftl::operators::Graph>(config(), "depth");
	depth_size_ = cv::Size(	config()->value("width", 1280),
							config()->value("height", 720));

	pipe_->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
	pipe_->append<ftl::operators::CrossSupport>("cross");
	#ifdef HAVE_OPTFLOW
	pipe_->append<ftl::operators::NVOpticalFlow>("optflow", Channel::Colour, Channel::Flow, Channel::Colour2, Channel::Flow2);
	//if (size == 1) pipe_->append<ftl::operators::OpticalFlowTemporalSmoothing>("optflow_filter", Channel::Disparity);
	#endif
	#ifdef HAVE_LIBSGM
	pipe_->append<ftl::operators::FixstarsSGM>("algorithm");
	#else
	pipe_->append<ftl::operators::StereoDisparity>("algorithm");
	#endif
	pipe_->append<ftl::operators::DisparityBilateralFilter>("bilateral_filter");
	//pipe_->append<ftl::operators::OpticalFlowTemporalSmoothing>("optflow_filter", Channel::Disparity);
	pipe_->append<ftl::operators::DisparityToDepth>("calculate_depth");
	#ifdef HAVE_OPTFLOW
	//pipe_->append<ftl::operators::OpticalFlowTemporalSmoothing>("optflow_filter", Channel::Depth);  // FIXME: Has a history so not with multiple sources!
	#endif
	pipe_->append<ftl::operators::Normals>("normals");  // Estimate surface normals
	pipe_->append<ftl::operators::DiscontinuityMask>("discontinuity_mask");
	pipe_->append<ftl::operators::AggreMLS>("mls");  // Perform MLS (using smoothing channel)
}

bool DepthChannel::apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) {
	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	rbuf_.resize(in.frames.size());

	for (size_t i=0; i<in.frames.size(); ++i) {
		if (!in.hasFrame(i)) continue;
		auto &f = in.frames[i];
		if (!f.hasChannel(Channel::Depth) && f.hasChannel(Channel::Right)) {
			_createPipeline(in.frames.size());

			cv::cuda::GpuMat& left = f.get<cv::cuda::GpuMat>(Channel::Left);
			cv::cuda::GpuMat& right = f.get<cv::cuda::GpuMat>(Channel::Right);
			cv::cuda::GpuMat& depth = f.create<cv::cuda::GpuMat>(Channel::Depth);
			depth.create(left.size(), CV_32FC1);

			if (left.empty() || right.empty()) continue;

			/*if (depth_size_ != left.size()) {
				auto &col2 = f.create<cv::cuda::GpuMat>(Channel::ColourHighRes);
				cv::cuda::resize(left, col2, depth_size_, 0.0, 0.0, cv::INTER_CUBIC, cvstream);
				f.createTexture<uchar4>(Channel::ColourHighRes, true);
				f.swapChannels(Channel::Colour, Channel::ColourHighRes);
			}

			if (depth_size_ != right.size()) {
				cv::cuda::resize(right, rbuf_[i], depth_size_, 0.0, 0.0, cv::INTER_CUBIC, cvstream);
				cv::cuda::swap(right, rbuf_[i]);
			}*/

			pipe_->apply(f, f, stream);
		}
	}

	return true;
}

bool DepthChannel::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	//rbuf_.resize(1);

	auto &f = in;
	if (!f.hasChannel(Channel::Depth) && f.hasChannel(Channel::Right)) {
		_createPipeline(1);

		cv::cuda::GpuMat& left = f.get<cv::cuda::GpuMat>(Channel::Left);
		cv::cuda::GpuMat& right = f.get<cv::cuda::GpuMat>(Channel::Right);
		cv::cuda::GpuMat& depth = f.create<cv::cuda::GpuMat>(Channel::Depth);
		depth.create(depth_size_, CV_32FC1);

		if (left.empty() || right.empty()) return false;

		/*if (depth_size_ != left.size()) {
			auto &col2 = f.create<cv::cuda::GpuMat>(Channel::ColourHighRes);
			cv::cuda::resize(left, col2, depth_size_, 0.0, 0.0, cv::INTER_CUBIC, cvstream);
			f.createTexture<uchar4>(Channel::ColourHighRes, true);
			f.swapChannels(Channel::Colour, Channel::ColourHighRes);
		}

		if (depth_size_ != right.size()) {
			cv::cuda::resize(right, rbuf_[i], depth_size_, 0.0, 0.0, cv::INTER_CUBIC, cvstream);
			cv::cuda::swap(right, rbuf_[i]);
		}*/

		pipe_->apply(f, f, stream);
	}

	return true;
}
