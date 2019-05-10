#ifndef _FTL_COMMON_OPENCV_TO_PCL_HPP_
#define _FTL_COMMON_OPENCV_TO_PCL_HPP_

#include <ftl/config.h>
#include <opencv2/opencv.hpp>

#if defined HAVE_PCL
#include <pcl/common/common_headers.h>
#endif  // HAVE_PCL

namespace ftl {
namespace utility {

/**
 * Convert an OpenCV point cloud matrix and RGB image to a PCL XYZRGB point cloud.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr matToPointXYZ(const cv::Mat &cvcloud, const cv::Mat &rgbimage);

};
};

#endif  // _FTL_COMMON_OPENCV_TO_PCL_HPP_
