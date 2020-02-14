#include "optimization.hpp"
#include "calibration.hpp"

#include "loguru.hpp"

#include <algorithm>
#include <unordered_set>

#include <opencv2/calib3d.hpp>
#include <ceres/rotation.h>

using std::vector;

using cv::Mat;

using cv::Point3d;
using cv::Point2d;

using cv::Rect;

using ftl::calibration::BundleAdjustment;
using ftl::calibration::Camera;

////////////////////////////////////////////////////////////////////////////////

struct ReprojectionError {
	/**
	 * Reprojection error.
	 *
	 * Camera model has _CAMERA_PARAMETERS parameters:
	 *
	 * - rotation and translation: rx, ry, rz, tx, ty, tx
	 * - focal length: f (fx == fy assumed)
	 * - pricipal point: cx, cy
	 * - first three radial distortion coefficients: k1, k2, k3
	 *
	 * Camera model documented in
	 * https://docs.opencv.org/master/d9/d0c/group__calib3d.html
	 */
	explicit ReprojectionError(double observed_x, double observed_y)
		: observed_x(observed_x), observed_y(observed_y) {}

	template <typename T>
	bool operator()(const T* const camera,
					const T* const point,
					T* residuals) const {

		T p[3];

		// Apply rotation and translation
		ceres::AngleAxisRotatePoint(camera + Camera::Parameter::ROTATION, point, p);

		p[0] += camera[Camera::Parameter::TX];
		p[1] += camera[Camera::Parameter::TY];
		p[2] += camera[Camera::Parameter::TZ];

		T x = T(p[0]) / p[2];
		T y = T(p[1]) / p[2];

		// Intrinsic parameters
		const T& focal = camera[Camera::Parameter::F];
		const T& cx = camera[Camera::Parameter::CX];
		const T& cy = camera[Camera::Parameter::CY];

		// Distortion parameters k1, k2, k3
		const T& k1 = camera[Camera::Parameter::K1];
		const T& k2 = camera[Camera::Parameter::K2];
		const T& k3 = camera[Camera::Parameter::K3];

		const T r2 = x*x + y*y;
		const T r4 = r2*r2;
		const T r6 = r4*r2;

		T distortion = T(1.0) + k1*r2 + k2*r4 + k3*r6;

		// Projected point position
		T predicted_x = focal*x*distortion + cx;
		T predicted_y = focal*y*distortion + cy;

		// Error: the difference between the predicted and observed position
		residuals[0] = predicted_x - T(observed_x);
		residuals[1] = predicted_y - T(observed_y);

		return true;
	}

	static ceres::CostFunction* Create(	const double observed_x,
										const double observed_y) {
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, _NCAMERA_PARAMETERS, 3>(
					new ReprojectionError(observed_x, observed_y)));
	}

	static ceres::CostFunction* Create(	const Point2d &observed) {
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, _NCAMERA_PARAMETERS, 3>(
					new ReprojectionError(observed.x, observed.y)));
	}

	double observed_x;
	double observed_y;
};

struct LengthError {
	explicit LengthError(const double d) : d(d) {}

	template <typename T>
	bool operator()(const T* const p1, const T* const p2, T* residual) const {
		auto x = p1[0] - p2[0];
		auto y = p1[1] - p2[1];
		auto z = p1[2] - p2[2];
		residual[0] = T(d) - sqrt(x*x + y*y + z*z);

		return true;
	}

	static ceres::CostFunction* Create(const double d) {
		return (new ceres::AutoDiffCostFunction<LengthError, 1, 3, 3>(new LengthError(d)));
	}

	double d;
};

struct ScaleError {
	ScaleError(const double d, const Point3d& p) : d(d), p(p) {}

	template <typename T>
	bool operator()(const T* const s, T* residual) const {
		auto x = T(p.x) * s[0];
		auto y = T(p.y) * s[0];
		auto z = T(p.z) * s[0];
		residual[0] = T(d) - sqrt(x*x + y*y + z*z);

		return true;
	}

	static ceres::CostFunction* Create(const double d, const Point3d p) {
		return (new ceres::AutoDiffCostFunction<ScaleError, 1, 1>(new ScaleError(d, p)));
	}

	double d;
	Point3d p;
};

////////////////////////////////////////////////////////////////////////////////

double ftl::calibration::optimizeScale(const vector<Point3d> &object_points, vector<Point3d> &points) {

	// use exceptions instead
	CHECK(points.size() % object_points.size() == 0);
	CHECK(points.size() % 2 == 0);

	// initial scale guess from first two object points

	double f = 0.0;
	double m = 0.0;

	for (size_t i = 0; i < points.size(); i += 2) {
		const Point3d &p1 = points[i];
		const Point3d &p2 = points[i+1];

		Point3d p = p1 - p2;

		double x = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
		f = f + 1.0;
		m = m + (x - m) / f;
	}

	double scale = norm(object_points[0]-object_points[1]) / m;

	// optimize using all distances between points

	vector<double> d;
	ceres::Problem problem;
	auto loss_function = new ceres::HuberLoss(1.0);

	// use all pairwise distances
	for (size_t i = 0; i < object_points.size(); i++) {
		for (size_t j = i + 1; j < object_points.size(); j++) {
			d.push_back(norm(object_points[i]-object_points[j]));
		}
	}

	for (size_t i = 0; i < points.size(); i += object_points.size()) {
		size_t i_d = 0;
		for (size_t i = 0; i < object_points.size(); i++) {
			for (size_t j = i + 1; j < object_points.size(); j++) {
				auto cost_function = ScaleError::Create(d[i_d++], points[i]-points[j]);
				problem.AddResidualBlock(cost_function, loss_function, &scale);
			}
		}
	}

	ceres::Solver::Options options;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	for (auto &p : points) { p = p * scale; }
	return scale;
}

////////////////////////////////////////////////////////////////////////////////

void BundleAdjustment::addCamera(Camera& camera) {
	// cameras can't be added after points
	if (points_.size() != 0) { throw std::exception(); }

	cameras_.push_back(&camera);
}

void BundleAdjustment::addCameras(vector<Camera>& cameras) {
	for (auto& camera : cameras) { addCamera(camera); }
}

void BundleAdjustment::addPoint(const vector<bool>& visibility, const vector<Point2d>& observations, Point3d& point) {
	if ((observations.size() != visibility.size()) ||
		(visibility.size() != cameras_.size())) { throw std::exception(); }

	points_.push_back(BundleAdjustment::Point{visibility, observations, reinterpret_cast<double*>(&point)});
}

void BundleAdjustment::addPoints(const vector<vector<bool>>& visibility, const vector<vector<Point2d>>& observations, vector<Point3d>& points) {
	if ((observations.size() != points.size()) ||
		(observations.size() != visibility.size())) { throw std::exception(); }

	for (size_t i = 0; i < points.size(); i++) {
		addPoint(visibility[i], observations[i], points[i]);
	}
}

void BundleAdjustment::addPoint(const vector<Point2d>& observations, Point3d& point) {
	vector<bool> visibility(observations.size(), true);
	addPoint(visibility, observations, point);
}
void BundleAdjustment::addPoints(const vector<vector<Point2d>>& observations, std::vector<Point3d>& points) {
	if (observations.size() != points.size()) { throw std::exception(); }
	for (size_t i = 0; i < points.size(); i++) {
		addPoint(observations[i], points[i]);
	}
}

void BundleAdjustment::addConstraintObject(const vector<Point3d> &object_points) {
	if (points_.size() % object_points.size() != 0) { throw std::exception(); }
	constraints_object_.push_back(BundleAdjustment::ConstraintObject {0, (int) points_.size(), object_points});
}

void BundleAdjustment::_buildBundleAdjustmentProblem(ceres::Problem &problem, const BundleAdjustment::Options &options) {

	ceres::LossFunction *loss_function = nullptr;

	if (options.loss == Options::Loss::HUBER) {
		loss_function = new ceres::HuberLoss(1.0);
	}
	else if (options.loss == Options::Loss::CAUCHY) {
		loss_function = new ceres::CauchyLoss(1.0);
	}

	for (auto point : points_) {
		for (size_t i = 0; i < point.observations.size(); i++) {
			if (!point.visibility[i]) { continue; }
			ceres::CostFunction* cost_function =
				ReprojectionError::Create(point.observations[i]);

			problem.AddResidualBlock(cost_function,
						loss_function,
						getCameraPtr(i),
						point.point);
		}
	}

	// apply options

	vector<int> constant_camera_parameters;

	// extrinsic paramters
	if (!options.optimize_motion) {
		for (int i = 0; i < 3; i++) {
			constant_camera_parameters.push_back(Camera::Parameter::ROTATION + i);
			constant_camera_parameters.push_back(Camera::Parameter::TRANSLATION + i);
		}
	}

	if (!options.fix_distortion) {
		LOG(WARNING) << "Optimization of distortion parameters is not supported"
					 << "and results may contain invalid values.";
	}

	// intrinsic parameters
	if (!options.optimize_intrinsic || options.fix_focal) {
		constant_camera_parameters.push_back(Camera::Parameter::F);
	}
	if (!options.optimize_intrinsic || options.fix_principal_point) {
		constant_camera_parameters.push_back(Camera::Parameter::CX);
		constant_camera_parameters.push_back(Camera::Parameter::CY);
	}

	if (!options.optimize_intrinsic || options.fix_distortion) {
		constant_camera_parameters.push_back(Camera::Parameter::K1);
		constant_camera_parameters.push_back(Camera::Parameter::K2);
		constant_camera_parameters.push_back(Camera::Parameter::K3);
	}

	if (!options.optimize_motion && !options.optimize_intrinsic) {
		// do not optimize any camera parameters
		for (size_t i = 0; i < cameras_.size(); i++) {
			problem.SetParameterBlockConstant(getCameraPtr(i));
		}
	}
	else {
		std::unordered_set<int> fix_extrinsic(
			options.fix_camera_extrinsic.begin(), options.fix_camera_extrinsic.end());

		std::unordered_set<int> fix_intrinsic(
			options.fix_camera_extrinsic.begin(), options.fix_camera_extrinsic.end());

		for (size_t i = 0; i < cameras_.size(); i++) {
			std::unordered_set<int> fixed_parameters(
				constant_camera_parameters.begin(),
				constant_camera_parameters.end());

			if (fix_extrinsic.find(i) != fix_extrinsic.end()) {
				fixed_parameters.insert({
					Camera::Parameter::RX, Camera::Parameter::RY,
					Camera::Parameter::RZ, Camera::Parameter::TX,
					Camera::Parameter::TY, Camera::Parameter::TZ
				});
			}

			if (fix_intrinsic.find(i) != fix_intrinsic.end()) {
				fixed_parameters.insert({
					Camera::Parameter::F, Camera::Parameter::CX,
					Camera::Parameter::CY
				});
			}

			vector<int> params(fixed_parameters.begin(), fixed_parameters.end());

			if (params.size() == _NCAMERA_PARAMETERS) {
				// Ceres crashes if all parameters are set constant using
				// SubsetParameterization()
				problem.SetParameterBlockConstant(getCameraPtr(i));
			}
			else if (params.size() > 0) {
				problem.SetParameterization(getCameraPtr(i),
					new ceres::SubsetParameterization(_NCAMERA_PARAMETERS, params));
			}
		}
	}

	if (!options.optmize_structure) {
		// do not optimize points
		for (auto &point : points_) { problem.SetParameterBlockConstant(point.point); }
	}
}

void BundleAdjustment::_buildLengthProblem(ceres::Problem &problem, const BundleAdjustment::Options &options) {

	for (auto &constraint : constraints_object_) {
		int npoints = constraint.object_points.size();
		auto &object_points = constraint.object_points;

		vector<double> d;
		for (int i = 0; i < npoints; i++) {
			for (int j = i + 1; j < npoints; j++) {
				d.push_back(norm(object_points[i]-object_points[j]));
			}
		}

		for (int p = constraint.idx_start; p < constraint.idx_end; p += npoints) {
			size_t i_d = 0;
			for (size_t i = 0; i < object_points.size(); i++) {
				for (size_t j = i + 1; j < object_points.size(); j++) {
					double* p1 = points_[p+i].point;
					double* p2 = points_[p+j].point;

					auto cost_function = LengthError::Create(d[i_d++]);

					problem.AddResidualBlock(cost_function, nullptr, p1, p2);
				}
			}
		}
	}
}

void BundleAdjustment::_buildProblem(ceres::Problem &problem, const BundleAdjustment::Options &options) {

	_buildBundleAdjustmentProblem(problem, options);
	_buildLengthProblem(problem, options);
}

void BundleAdjustment::run(const BundleAdjustment::Options &bundle_adjustment_options) {
	ceres::Problem problem;
	_buildProblem(problem, bundle_adjustment_options);

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = bundle_adjustment_options.verbose;
	
	if (bundle_adjustment_options.max_iter > 0) {
		options.max_num_iterations = bundle_adjustment_options.max_iter;
	}
	
	if (bundle_adjustment_options.num_threads > 0) {
		options.num_threads = bundle_adjustment_options.num_threads;
	}

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	if (bundle_adjustment_options.verbose) {
		LOG(INFO) << summary.BriefReport();
	}
	else {
		DLOG(INFO) << summary.BriefReport();
	}
}

void BundleAdjustment::run() {
	BundleAdjustment::Options options;
	run(options);
}

double BundleAdjustment::reprojectionError(int camera) {
	vector<Point2d>	observations;
	vector<Point3d>	points;

	observations.reserve(points_.size());
	points.reserve(points_.size());

	for (const auto& point : points_) {
		if (!point.visibility[camera]) { continue; }
		observations.push_back(point.observations[camera]);
		points.push_back(Point3d(point.point[0], point.point[1], point.point[2]));
	}

	auto K = cameras_[camera]->intrinsicMatrix();
	auto rvec = cameras_[camera]->rvec();
	auto tvec = cameras_[camera]->tvec();

	return ftl::calibration::reprojectionError(observations, points, K, Mat::zeros(1, 5, CV_64FC1), rvec, tvec);
}

double BundleAdjustment::reprojectionError() {
	double error = 0.0;
	for (size_t i = 0; i < cameras_.size(); i++) { error += reprojectionError(i); }
	return error * (1.0 / (double) cameras_.size());
}
