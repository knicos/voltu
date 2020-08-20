#pragma once

#include <ftl/config.h>
#include <ftl/operators/operator.hpp>

#ifdef HAVE_OPTFLOW
#include <opencv2/cudaoptflow.hpp>
#endif

#include <opencv2/cudastereo.hpp>
#include <opencv2/cudafilters.hpp>

#ifdef HAVE_LIBSGM
#include <libsgm.h>
#include <opencv2/cudaimgproc.hpp>
#endif

namespace ftl {
namespace operators {

class StereoDisparity : public ftl::operators::Operator {
public:
	StereoDisparity(ftl::operators::Graph *g, ftl::Configurable* cfg);

	~StereoDisparity();
	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	bool isMemoryHeavy() const override { return true; }

private:
	bool init();

	struct Impl;
	Impl *impl_;

	cv::cuda::GpuMat disp32f_;
};

#ifdef HAVE_LIBSGM
/*
 * FixstarsSGM https://github.com/fixstars/libSGM
 *
 * Requires modified version in lib/libsgm
 */
class FixstarsSGM : public ftl::operators::Operator {
	public:
	FixstarsSGM(ftl::operators::Graph *g, ftl::Configurable* cfg);

	~FixstarsSGM();
	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	bool isMemoryHeavy() const override { return true; }

	private:
	bool init();
	bool updateParameters();
	bool updateP2Parameters();
	void computeP2(cudaStream_t &stream);
	void _variance_mask(cv::InputArray in, cv::OutputArray out, int wsize, cv::cuda::Stream &cvstream);

	sgm::StereoSGM *ssgm_;
	cv::Size size_;
	cv::cuda::GpuMat lbw_;
	cv::cuda::GpuMat rbw_;
	cv::cuda::GpuMat lbw_full_;
	cv::cuda::GpuMat rbw_full_;
	cv::cuda::GpuMat disp_int_;

	cv::cuda::GpuMat P2_map_;
	cv::cuda::GpuMat weights_;
	cv::cuda::GpuMat weightsF_;
	cv::cuda::GpuMat edges_;
	cv::Ptr<cv::cuda::CannyEdgeDetector> canny_;
	cv::Ptr<cv::cuda::Filter> filter_;

	cv::cuda::GpuMat im_;
	cv::cuda::GpuMat im2_;
	cv::cuda::GpuMat mean_;
	cv::cuda::GpuMat mean2_;

	int P1_;
	int P2_;
	int max_disp_;
	float uniqueness_;
	bool use_P2_map_;
};
#endif

class DisparityBilateralFilter : public::ftl::operators::Operator {
	public:
	DisparityBilateralFilter(ftl::operators::Graph *g, ftl::Configurable*);

	~DisparityBilateralFilter() {};

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	cv::Ptr<cv::cuda::DisparityBilateralFilter> filter_;
	cv::cuda::GpuMat disp_int_;
	cv::cuda::GpuMat disp_int_result_;
	cv::cuda::GpuMat rgb_;
	double scale_;
	int radius_;
	int iter_;
	int n_disp_;
};

/*
 * Calculate depth from disparity
 */
class DisparityToDepth : public ftl::operators::Operator {
	public:
	DisparityToDepth(ftl::operators::Graph *g, ftl::Configurable* cfg) :
		ftl::operators::Operator(g, cfg) {}

	~DisparityToDepth() {};
	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;
};

/**
 * Create a missing depth channel using Left and Right colour with a complete
 * disparity pipeline. This is a wrapper operator that combines all of the
 * disparity to depth steps.
 */
class DepthChannel : public ftl::operators::Operator {
	public:
	DepthChannel(ftl::operators::Graph *g, ftl::Configurable *cfg);
	~DepthChannel();

	inline Operator::Type type() const override { return Operator::Type::ManyToMany; }

	bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) override;
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	ftl::operators::Graph *pipe_;
	std::vector<cv::cuda::GpuMat> rbuf_;
	cv::Size depth_size_;

	void _createPipeline(size_t);
};

/*
 * Optical flow smoothing for depth
 */
#ifdef HAVE_OPTFLOW
class OpticalFlowTemporalSmoothing : public ftl::operators::Operator {
	public:
	OpticalFlowTemporalSmoothing(ftl::operators::Graph *g, ftl::Configurable*);
	OpticalFlowTemporalSmoothing(ftl::operators::Graph *g, ftl::Configurable*, const std::tuple<ftl::codecs::Channel> &params);
	~OpticalFlowTemporalSmoothing();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	void _init(ftl::Configurable* cfg);
	bool init();

	ftl::codecs::Channel channel_ = ftl::codecs::Channel::Depth;
	cv::cuda::GpuMat history_;
	cv::Size size_;
	int n_max_;
	float threshold_;
};
#endif

}
}
