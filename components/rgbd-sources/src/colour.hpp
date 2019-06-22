#ifndef _FTL_RGBD_COLOUR_HPP_
#define _FTL_RGBD_COLOUR_HPP_

#include <opencv2/opencv.hpp>

namespace ftl {
namespace rgbd {

void colourCorrection(cv::Mat &img, float gamma, int temp);
// void colourCorrection(cv::GpuMat &img, float gamma, int temp);

}
}

#endif  // _FTL_RGBD_COLOUR_HPP_
