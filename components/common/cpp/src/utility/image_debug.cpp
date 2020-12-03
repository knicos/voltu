/**
 * @file image_debug.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/utility/image_debug.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/highgui.hpp>

void ftl::utility::show_image(
	const cv::cuda::GpuMat &m,
	const std::string &name,
	float scale,
	ImageVisualisation iv
) {
	cv::Mat tmp;
	m.download(tmp);

	if (scale != 1.0f) {
		cv::resize(tmp, tmp, cv::Size(float(tmp.cols)*scale, float(tmp.rows)*scale));
	}

	// Must do some conversion.
	if (tmp.type() == CV_32F) {

	}

	if (iv == ftl::utility::ImageVisualisation::HEAT_MAPPED) {
		#if OPENCV_VERSION >= 40102
		cv::applyColorMap(tmp, tmp, cv::COLORMAP_TURBO);
		#else
		cv::applyColorMap(tmp, tmp, cv::COLORMAP_INFERNO);
		#endif
	}

	cv::imshow(name, tmp);
	cv::waitKey(1);
}
