#pragma once

#include <ftl/config.h>
#include <ftl/operators/operator.hpp>

#ifdef HAVE_OPTFLOW
#include <opencv2/cudaoptflow.hpp>
#endif

#include <opencv2/cudastereo.hpp>

#ifdef HAVE_LIBSGM
#include <libsgm.h>
#endif

namespace ftl {
namespace operators {

#ifdef HAVE_LIBSGM
/*
 * FixstarsSGM https://github.com/fixstars/libSGM
 * 
 * Requires modified version https://gitlab.utu.fi/joseha/libsgm
 */
class FixstarsSGM : public ftl::operators::Operator {
	public:
	explicit FixstarsSGM(ftl::Configurable* cfg);

	~FixstarsSGM();
	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	bool isMemoryHeavy() const override { return true; }
	
	private:
	bool init();
	bool updateParameters();
	
	sgm::StereoSGM *ssgm_;
	cv::Size size_;
	cv::cuda::GpuMat lbw_;
	cv::cuda::GpuMat rbw_;
	cv::cuda::GpuMat disp_int_;

	int P1_;
	int P2_;
	int max_disp_;
	float uniqueness_;
};
#endif

class DisparityBilateralFilter : public::ftl::operators::Operator {
	public:
	explicit DisparityBilateralFilter(ftl::Configurable*);

	~DisparityBilateralFilter() {};

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	cv::Ptr<cv::cuda::DisparityBilateralFilter> filter_;
	cv::cuda::GpuMat disp_int_;
	cv::cuda::GpuMat disp_int_result_;
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
	explicit DisparityToDepth(ftl::Configurable* cfg) :
		ftl::operators::Operator(cfg) {}

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
    explicit DepthChannel(ftl::Configurable *cfg);
    ~DepthChannel();

	inline Operator::Type type() const override { return Operator::Type::ManyToMany; }

    bool apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) override;
	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

    private:
    ftl::operators::Graph *pipe_;
	std::vector<cv::cuda::GpuMat> rbuf_;
	cv::Size depth_size_;

	void _createPipeline();
};

/*
 * Optical flow smoothing for depth
 */
#ifdef HAVE_OPTFLOW
class OpticalFlowTemporalSmoothing : public ftl::operators::Operator {
	public:
	explicit OpticalFlowTemporalSmoothing(ftl::Configurable*);
	OpticalFlowTemporalSmoothing(ftl::Configurable*, const std::tuple<ftl::codecs::Channel> &params);
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
