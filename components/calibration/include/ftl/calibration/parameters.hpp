#pragma once
#ifndef _FTL_CALIBRATION_PARAMETERS_HPP_
#define _FTL_CALIBRATION_PARAMETERS_HPP_

#include <opencv2/core/core.hpp>

namespace ftl {
namespace calibration {

/**
 * Camera paramters
 */
struct Camera {
	Camera() {}
	Camera(const cv::Mat& K, const cv::Mat& D, const cv::Mat& R, const cv::Mat& tvec);

	void setRotation(const cv::Mat& R);
	void setTranslation(const cv::Mat& tvec);
	void setExtrinsic(const cv::Mat& R, const cv::Mat& t) {
		setRotation(R);
		setTranslation(t);
	}

	void setIntrinsic(const cv::Mat& K);
	void setDistortion(const cv::Mat &D);
	void setIntrinsic(const cv::Mat& K, const cv::Mat& D) {
		setIntrinsic(K);
		setDistortion(D);
	}

	cv::Mat intrinsicMatrix() const;
	cv::Mat distortionCoefficients() const;

	cv::Mat rvec() const;
	cv::Mat tvec() const;
	cv::Mat rmat() const;

	cv::Mat extrinsicMatrix() const;
	cv::Mat extrinsicMatrixInverse() const;

	const static int n_parameters = 12;
	const static int n_distortion_parameters = 3;

	double data[n_parameters] = {0.0};

	enum Parameter {
		ROTATION = 0,
		RX = 0,
		RY = 1,
		RZ = 2,
		TRANSLATION = 3,
		TX = 3,
		TY = 4,
		TZ = 5,
		F = 6,
		CX = 7,
		CY = 8,
		DISTORTION = 9,
		K1 = 9,
		K2 = 10,
		K3 = 11
	};
};

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


namespace transform {

// TODO: Some of the methods can be directly replace with OpenCV
//       (opencv2/calib3d.hpp)

/**
 * @brief Get rotation matrix and translation vector from transformation matrix.
 * @param T    transformation matrix (4x4) or (3x4)
 * @param R    (out) rotation matrix (3x3)
 * @param t    (out) translation vector (3x1)
 */
inline void getRotationAndTranslation(const cv::Mat &T, cv::Mat &R, cv::Mat &t) {
	T(cv::Rect(0, 0, 3, 3)).copyTo(R);
	T(cv::Rect(3, 0, 1, 3)).copyTo(t);
}

/**
 * @brief Inverse transform inplace
 * @param R   rotation matrix (3x3)
 * @param t   translation vector (3x1)
 */
inline void inverse(cv::Mat &R, cv::Mat &t) {
	cv::Mat t_(3, 1, CV_64FC1);

	t_.at<double>(0) = -R.col(0).dot(t);
	t_.at<double>(1) = -R.col(1).dot(t);
	t_.at<double>(2) = -R.col(2).dot(t);

	cv::swap(t_, t);
	R = R.t();
}

/**
 * @brief Inverse transform inplace
 * @param T   transformation matrix (4x4)
 */
inline void inverse(cv::Mat &T) {
	cv::Mat rmat;
	cv::Mat tvec;
	getRotationAndTranslation(T, rmat, tvec);
	T = cv::Mat::eye(4, 4, CV_64FC1);

	T(cv::Rect(3, 0, 1, 1)) = -rmat.col(0).dot(tvec);
	T(cv::Rect(3, 1, 1, 1)) = -rmat.col(1).dot(tvec);
	T(cv::Rect(3, 2, 1, 1)) = -rmat.col(2).dot(tvec);
	T(cv::Rect(0, 0, 3, 3)) = rmat.t();
}

inline cv::Point3d apply(const cv::Point3d& point, const cv::InputArray& R, const cv::InputArray& t) {
	cv::Mat R_ = R.getMat();
	cv::Mat t_ = t.getMat();
	cv::Mat p_new = R_ * cv::Mat(point) + t_;
	return cv::Point3d(p_new.at<double>(0), p_new.at<double>(1), p_new.at<double>(2));
}

inline cv::Point3d apply(const cv::Point3d& point, const cv::InputArray& T) {
	cv::Mat T_ = T.getMat();
	cv::Mat rmat;
	cv::Mat tvec;
	getRotationAndTranslation(T_, rmat, tvec);
	return apply(point, rmat, tvec);
}

} // namespace transform

/**
 * @brief Scale camera intrinsic matrix
 * @param size_new	New resolution
 * @param size_old	Original (camera matrix) resolution
 */
cv::Mat scaleCameraMatrix(const cv::Mat &K, const cv::Size &size_new, const cv::Size &size_old);

/**
 * @brief Calculate MSE reprojection error
 * @param points_im points in pixel coordinates
 * @param points    points in camera coordinates
 * @param K         intrinsic matrix
 * @param D         distortion coefficients
 * @param R         rotation matrix or vector
 * @param t         translation vector
 */
double reprojectionError(const std::vector<cv::Point2d>& points_im,
	const std::vector<cv::Point3d>& points, const cv::Mat& K, const cv::Mat& D,
	const cv::Mat& R, const cv::Mat& t);

}
}

#endif
