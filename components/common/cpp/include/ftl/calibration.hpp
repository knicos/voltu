#pragma once
#ifndef _FTL_COMMON_CALIBRATION_HPP_
#define _FTL_COMMON_CALIBRATION_HPP_

#include <opencv2/core/core.hpp>

namespace ftl {
namespace calibration {

namespace validate {

/**
 * @brief Valid translation for stereo camera.
 */
bool translationStereo(const cv::Mat &t);

bool rotationMatrix(const cv::Mat &M);

bool pose(const cv::Mat &M);

bool cameraMatrix(const cv::Mat &M);

/**
 * @brief Check if D contains valid distortion coefficients.
 * @param D    distortion coefficients
 * @param size resolution
 * @note Tangential and prism distortion coefficients are not validated.
 * 
 * Radial distortion is always monotonic for real lenses and distortion
 * function has to be bijective. This is verified by evaluating the distortion
 * function for integer values from 0 to sqrt(width^2+height^2).
 * 
 * Camera model documented in
 * https://docs.opencv.org/master/d9/d0c/group__calib3d.html#details
 */
bool distortionCoefficients(const cv::Mat &D, cv::Size size);

}

/**
 * @brief Scale camera intrinsic matrix
 * @param size_new	New resolution
 * @param size_old	Original (camera matrix) resolution
 */
cv::Mat scaleCameraMatrix(const cv::Mat &K, const cv::Size &size_new, const cv::Size &size_old);

}
}

#endif
