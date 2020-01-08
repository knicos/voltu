#include <ftl/operators/disparity.hpp>

#include "ftl/operators/smoothing.hpp"
#include "ftl/operators/colours.hpp"
#include "ftl/operators/normals.hpp"
#include "ftl/operators/filling.hpp"
#include "ftl/operators/segmentation.hpp"
#include "ftl/operators/disparity.hpp"
#include "ftl/operators/mask.hpp"

using ftl::operators::DepthChannel;
using ftl::codecs::Channel;

DepthChannel::DepthChannel(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {
	pipe_ = nullptr;
}

DepthChannel::~DepthChannel() {

}

void DepthChannel::_createPipeline() {
	if (pipe_ != nullptr) return;

	pipe_ = ftl::config::create<ftl::operators::Graph>(config(), "depth");
	depth_size_ = cv::Size(	config()->value("width", 1280),
							config()->value("height", 720));

	pipe_->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
	pipe_->append<ftl::operators::FixstarsSGM>("algorithm");
	#ifdef HAVE_OPTFLOW
	pipe_->append<ftl::operators::OpticalFlowTemporalSmoothing>("optflow_filter");
	#endif
	pipe_->append<ftl::operators::DisparityBilateralFilter>("bilateral_filter");
	pipe_->append<ftl::operators::DisparityToDepth>("calculate_depth");
	pipe_->append<ftl::operators::Normals>("normals");  // Estimate surface normals
	pipe_->append<ftl::operators::CrossSupport>("cross");
	pipe_->append<ftl::operators::DiscontinuityMask>("discontinuity_mask");
	pipe_->append<ftl::operators::AggreMLS>("mls");  // Perform MLS (using smoothing channel)
}

bool DepthChannel::apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) {
	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream);

	rbuf_.resize(in.frames.size());

	for (int i=0; i<in.frames.size(); ++i) {
		auto &f = in.frames[i];
		if (!f.hasChannel(Channel::Depth) && f.hasChannel(Channel::Right)) {
			_createPipeline();

			cv::cuda::GpuMat& left = f.get<cv::cuda::GpuMat>(Channel::Left);
			cv::cuda::GpuMat& right = f.get<cv::cuda::GpuMat>(Channel::Right);
			cv::cuda::GpuMat& depth = f.create<cv::cuda::GpuMat>(Channel::Depth);
			depth.create(depth_size_, CV_32FC1);

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

			pipe_->apply(f, f, in.sources[i], stream);
		}
	}

	return true;
}
