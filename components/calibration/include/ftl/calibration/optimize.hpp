#pragma once
#ifndef _FTL_CALIBRATION_OPTIMIZE_HPP_
#define _FTL_CALIBRATION_OPTIMIZE_HPP_

#include <vector>
#include <functional>

#include "parameters.hpp"

#include <ftl/config.h>

#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>

// BundleAdjustment uses Point3d instances via double*
static_assert(sizeof(cv::Point2d) == 2*sizeof(double));
static_assert(std::is_standard_layout<cv::Point2d>());
static_assert(sizeof(cv::Point3d) == 3*sizeof(double));
static_assert(std::is_standard_layout<cv::Point3d>());

namespace ftl {
namespace calibration {

#ifdef HAVE_CERES

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
 *
 * @note: Distortion paramters are used in reprojection error, but they are
 *        not not optimized.
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
		 *       included in the model, thus distortion parameters are always
		 *       fixed.
		 */
		// distortion coefficient optimization is not supported
		bool fix_distortion = true;

		bool optimize_intrinsic = true;
		bool optimize_motion = true;
		bool optmize_structure = true;

		int num_threads = -1;
		int max_iter = -1;
		bool verbose = false;
	};

	/**
	 * Add camera(s)
	 */
	void addCamera(Camera &K);
	void addCameras(std::vector<Camera> &K);

	/**
	 * @brief Add points
	 */
	void addPoint(const std::vector<bool>& visibility, const std::vector<cv::Point2d>& observations, cv::Point3d& point);
	void addPoints(const std::vector<std::vector<bool>>& visibility, const std::vector<std::vector<cv::Point2d>>& observations,
		std::vector<cv::Point3d>& points);

	/**
	 * @brief Add points, all assumed visible
	 */
	void addPoint(const std::vector<cv::Point2d>& observations, cv::Point3d& point);
	void addPoints(const std::vector<std::vector<cv::Point2d>>& observations, std::vector<cv::Point3d>& points);

	void addConstraintPlane(int group_size);
	void addConstraintObject(const std::vector<cv::Point3d>& object_points);

	/** @brief Perform bundle adjustment with custom options.
	 */
	void run(const BundleAdjustment::Options& options);

	/** @brief Perform bundle adjustment using default options
	 */
	void run();

	/** @brief Calculate MSE error (for one camera)
	 */
	double reprojectionError(int camera);

	/** @brief Calculate MSE error for all cameras
	 */
	double reprojectionError();

protected:
	double* getCameraPtr(int i) { return cameras_[i]->data; }

	void _buildProblem(ceres::Problem& problem, const BundleAdjustment::Options& options);
	void _buildBundleAdjustmentProblem(ceres::Problem& problem, const BundleAdjustment::Options& options);
	void _buildLengthProblem(ceres::Problem& problem, const BundleAdjustment::Options& options);

private:
	// point to be optimized and corresponding observations
	struct Point {
		std::vector<bool> visibility;

		// pixel coordinates: x, y
		std::vector<cv::Point2d> observations;

		// world coordinates: x, y, z
		double* point;
	};

	// object shape based constraint for group of points from idx_start to idx_end
	struct ConstraintObject {
		int idx_start;
		int idx_end;
		std::vector<cv::Point3d> object_points;
	};

	// planar constraint for group of points from idx_start to idx_end
	struct ConstraintPlane {
		int idx_start;
		int idx_end;
		int group_size;
	};

	// camera paramters (as pointers)
	std::vector<Camera*> cameras_;
	std::vector<BundleAdjustment::Point> points_;
	std::vector<BundleAdjustment::ConstraintObject> constraints_object_;
};

#endif

}
}

#endif
