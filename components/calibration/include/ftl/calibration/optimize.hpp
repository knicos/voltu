#pragma once
#ifndef _FTL_CALIBRATION_OPTIMIZE_HPP_
#define _FTL_CALIBRATION_OPTIMIZE_HPP_

#include <vector>
#include <functional>

#include "parameters.hpp"

#include <ftl/config.h>

#include <opencv2/core/core.hpp>

// BundleAdjustment uses Point3d instances via double*
static_assert(sizeof(cv::Point2d) == 2*sizeof(double));
static_assert(std::is_standard_layout<cv::Point2d>());
static_assert(sizeof(cv::Point3d) == 3*sizeof(double));
static_assert(std::is_standard_layout<cv::Point3d>());

namespace ceres {
	struct Problem;
}

namespace ftl {
namespace calibration {

/**
 * Camera paramters (Ceres)
 */
struct Camera {
	Camera() {}
	Camera(const cv::Mat& K, const cv::Mat& D, const cv::Mat& R, const cv::Mat& tvec, cv::Size size);
	Camera(const CalibrationData::Calibration& calib);

	CalibrationData::Intrinsic intrinsic() const;
	CalibrationData::Extrinsic extrinsic() const;

	void setRotation(const cv::Mat& R);
	void setTranslation(const cv::Mat& tvec);
	void setExtrinsic(const cv::Mat& R, const cv::Mat& t) {
		setRotation(R);
		setTranslation(t);
	}

	void setIntrinsic(const cv::Mat& K, cv::Size sz);
	void setDistortion(const cv::Mat &D);
	void setIntrinsic(const cv::Mat& K, const cv::Mat& D, cv::Size sz) {
		setIntrinsic(K, sz);
		setDistortion(D);
	}

	cv::Mat intrinsicMatrix() const;
	cv::Mat distortionCoefficients() const;

	cv::Mat rvec() const;
	cv::Mat tvec() const;
	cv::Mat rmat() const;

	cv::Mat extrinsicMatrix() const;
	cv::Mat extrinsicMatrixInverse() const;

	void toQuaternion();
	void toAngleAxis();

	cv::Size size;

	const static int n_parameters = 18;
	const static int n_distortion_parameters = 8;

	bool quaternion = false;
	double data[n_parameters] = {0.0};

	enum Parameter {
		ROTATION = 0,
		Q1 = 0,
		Q2 = 1,
		Q3 = 2,
		Q4 = 3,
		TRANSLATION = 4,
		TX = 4,
		TY = 5,
		TZ = 6,
		F = 7,
		CX = 8,
		CY = 9,
		DISTORTION = 10,
		K1 = 10,
		K2 = 11,
		P1 = 12,
		P2 = 13,
		K3 = 14,
		K4 = 15,
		K5 = 16,
		K6 = 17
	};
};

#ifdef HAVE_CERES

/** Project point using camera model implemented for Ceres */
cv::Point2d projectPoint(const Camera& camera, const cv::Point3d &p);

/**
 * @brief Optimize scale.
 * @param object Reference object points
 * @param points Points in camera coordinates, points.size() % objects.size() == 0
 * @returns Scale factor
 *
 * Finds scale factor s which minimizes error between relative distances of points.
 */
double optimizeScale(const std::vector<cv::Point3d>& object, std::vector<cv::Point3d>& points);

/**
 * Bundle adjustment. Optimizes parameters (Camera) and structure (Point3d) in
 * place.
 *
 * Parameters of camera model:
 * - rotation and translation rx, ry, rz, tx, ty, tz,
 * - focal legth and principal point: f, cx, cy
 * - radial distortion (first three cofficients): k1, k2, k3
 */
class BundleAdjustment {
public:
	/**
	 * Bundle adjustment options.
	 */
	struct Options {
		enum Loss {
			SQUARED,
			HUBER,
			CAUCHY,
		};

		Loss loss = Loss::SQUARED;

		bool use_nonmonotonic_steps = false;

		// use quaternion rotation
		bool use_quaternion = false;

		// fix_camera_extrinsic and fix_camera_intrinsic overlap with some of
		// the generic options. The more generic setting is always used, the
		// specific extrinsic/intrinsic options are applied on top of those.

		// fix extrinsic paramters for cameras
		std::vector<int> fix_camera_extrinsic = {};

		// fix intrinsic paramters for cameras
		std::vector<int> fix_camera_intrinsic = {};

		bool fix_focal = false;
		bool fix_principal_point = false;

		/**
		 * @todo Radial distortion must be monotonic. This constraint is not
		 *       included in the model.
		 */
		/// fix all distortion coefficients to constant (initial values)
		bool fix_distortion = true;
		/// use distortion coefficients k4, k5, and k6; if false, set to zero
		bool rational_model = true;
		/// distortion set to zero
		bool zero_distortion = false;

		bool optimize_intrinsic = true;
		bool optimize_motion = true;
		bool optmize_structure = true;

		int num_threads = -1;
		int max_iter = -1;
		bool verbose = false;
	};

	/**
	 * Add camera(s). Stored as pointers. TODO: copy instead
	 */
	void addCamera(Camera &K);
	void addCameras(std::vector<Camera> &K);

	/**
	 * @brief Add points
	 */
	void addPoint(const std::vector<bool>& visibility, const std::vector<cv::Point2d>& observations, cv::Point3d& point);
	/**
	 * @brief Vector for each camera TODO: verify this works
	 */
	void addPoints(const std::vector<std::vector<bool>>& visibility, const std::vector<std::vector<cv::Point2d>>& observations,
		std::vector<cv::Point3d>& points);

	/**
	 * @brief Add points, all assumed visible. Values copied.
	 */
	void addPoint(const std::vector<cv::Point2d>& observations, cv::Point3d& point);
	void addPoints(const std::vector<std::vector<cv::Point2d>>& observations, std::vector<cv::Point3d>& points);

	/** TODO: estimate pose for each view which to optimize */
	void addObject(const std::vector<cv::Point3d>& object_points);

	/** @brief Perform bundle adjustment with custom options.
	 */
	void run(const BundleAdjustment::Options& options);

	/**  @brief Get optimized points
	 *
	*/
	std::vector<cv::Point3d> getPoints();

	/** @brief Perform bundle adjustment using default options
	 */
	void run();

	/** @brief Calculate RMSE error (for one camera)
	 */
	double reprojectionError(const int camera) const;

	/** @brief Calculate RMSE error for all cameras
	 */
	double reprojectionError() const;

	/**/
	int removeObservations(double threshold);

protected:
	double* getCameraPtr(int i) { return cameras_.at(i)->data; }

	/** @brief Calculate squared error
	 */
	void _reprojectionErrorSE(const int camera, double &error, double &npoints) const;

	/** @brief Set camera parametrization (fixed parameters/cameras)
	 */
	void _setCameraParametrization(ceres::Problem& problem, const BundleAdjustment::Options& options);
	void _setStructureParametrization(ceres::Problem& problem, const BundleAdjustment::Options& options);

	void _buildProblem(ceres::Problem& problem, const BundleAdjustment::Options& options);
	void _buildBundleAdjustmentProblem(ceres::Problem& problem, const BundleAdjustment::Options& options);

	// remove?
	void _buildLengthProblem(ceres::Problem& problem, const BundleAdjustment::Options& options);

private:
	// point to be optimized and corresponding observations
	struct Point {
		std::vector<bool> visibility;

		// pixel coordinates: x, y
		std::vector<cv::Point2d> observations;

		// point in world coordinates
		cv::Point3d point;
	};

	// group of points with known structure; from idx_start to idx_end
	struct Object {
		int idx_start;
		int idx_end;
		std::vector<cv::Point3d> object_points;
	};

	// camera paramters (as pointers)
	std::vector<Camera*> cameras_;
	std::vector<BundleAdjustment::Point> points_;
	std::vector<BundleAdjustment::Object> objects_;
};

#endif

}
}

#endif
