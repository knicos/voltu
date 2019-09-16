#include "ftl/offilter.hpp"
#include "cuda_algorithms.hpp"

#ifdef HAVE_OPTFLOW

#include <loguru.hpp>

using namespace ftl::rgbd;

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
	const cv::cuda::GpuMat &optflow = frame.getChannel<cv::cuda::GpuMat>(kChanFlow, stream);
	frame.getChannel<cv::cuda::GpuMat>(kChanDisparity, stream);
	stream.waitForCompletion();
	if (optflow.empty()) { return; }

	cv::cuda::GpuMat &disp = frame.setChannel<cv::cuda::GpuMat>(kChanDisparity);
	ftl::cuda::optflow_filter(disp, optflow, disp_old_, n_max_, threshold_, stream);
}

#endif  // HAVE_OPTFLOW
