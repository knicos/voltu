#include "ftl/offilter.hpp"
#include "cuda_algorithms.hpp"

#ifdef HAVE_OPTFLOW

#include <loguru.hpp>

using namespace ftl::rgbd;
using namespace ftl::codecs;

using cv::Mat;
using cv::Size;

using std::vector;

template<typename T> static bool inline isValidDisparity(T d) { return (0.0 < d) && (d < 256.0); } // TODO

OFDisparityFilter::OFDisparityFilter(Size size, int n_frames, float threshold) :
	n_max_(n_frames + 1), threshold_(threshold)
{
	CHECK((n_max_ > 1) && (n_max_ <= 32)) << "History length must be between 0 and 31!";
	disp_old_ = cv::cuda::GpuMat(cv::Size(size.width * n_max_, size.height), CV_32FC1);
	
	/*nvof_ = cv::cuda::NvidiaOpticalFlow_1_0::create(size.width, size.height,
													cv::cuda::NvidiaOpticalFlow_1_0::NV_OF_PERF_LEVEL_SLOW,
													true, false, false, 0);*/
	
}

void OFDisparityFilter::filter(ftl::rgbd::Frame &frame, cv::cuda::Stream &stream)
{
	frame.upload(Channel::Flow, stream);
	const cv::cuda::GpuMat &optflow = frame.get<cv::cuda::GpuMat>(Channel::Flow);
	//frame.get<cv::cuda::GpuMat>(Channel::Disparity);
	stream.waitForCompletion();
	if (optflow.empty()) { return; }

	cv::cuda::GpuMat &disp = frame.create<cv::cuda::GpuMat>(Channel::Disparity);
	ftl::cuda::optflow_filter(disp, optflow, disp_old_, n_max_, threshold_, stream);
}

void OFDisparityFilter::filter(cv::cuda::GpuMat &disp, cv::cuda::GpuMat &optflow, cv::cuda::Stream &stream)
{
	if (disp.type != CV_32FC1) {
		LOG(ERROR) << "Optical flow filter expects CV_32FC1 (TODO)";
		return;
	}

	ftl::cuda::optflow_filter(disp, optflow, disp_old_, n_max_, threshold_, stream);
}

#endif  // HAVE_OPTFLOW
