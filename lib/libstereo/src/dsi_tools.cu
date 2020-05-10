#include <opencv2/core/cuda/common.hpp>
#include "dsi_tools.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

static void colorize(const cv::Mat &disp, cv::Mat &out, int ndisp=-1) {
	double dmin, dmax;
	cv::minMaxLoc(disp, &dmin, &dmax);
	cv::Mat dispf, dispc;
	disp.convertTo(dispf, CV_32FC1);
	dispf.convertTo(dispc, CV_8UC1, (1.0f / (ndisp > 0 ? (float) ndisp : dmax)) * 255.0f);

	#if OPENCV_VERSION >= 40102
	cv::applyColorMap(dispc, out, cv::COLORMAP_TURBO);
	#else
	cv::applyColorMap(dispc, out, cv::COLORMAP_INFERNO);
	#endif
}

void show_dsi_slice(cv::InputArray in) {
    cv::Mat dsitmp;

    if (in.isGpuMat()) {
        in.getGpuMat().download(dsitmp);
    } else {
        dsitmp = in.getMat();
    }
	float scale = 1280.0f / float(dsitmp.cols);
	cv::resize(dsitmp, dsitmp, cv::Size(dsitmp.cols*scale, dsitmp.rows*scale));
	colorize(dsitmp,dsitmp);
	cv::imshow("DSI Slice", dsitmp);
}
