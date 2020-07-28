#include "ftl/calibration/optimize.hpp"
#include "ftl/calibration/parameters.hpp"

#include <loguru.hpp>

#include <ftl/exception.hpp>

#include <algorithm>
#include <unordered_set>

#include <opencv2/calib3d.hpp>
#include <ceres/rotation.h>

using std::vector;

using cv::Mat;

using cv::Point3d;
using cv::Point2d;
using cv::Vec3d;
using cv::Size;
using cv::Rect;

using ftl::calibration::BundleAdjustment;
using ftl::calibration::Camera;

////////////////////////////////////////////////////////////////////////////////

void Camera::setRotation(const Mat& R) {
	if (((R.size() != Size(3, 3)) &&
		(R.size() != Size(3, 1)) &&
		(R.size() != Size(1, 3))) ||
		(R.type() != CV_64FC1)) {

		throw ftl::exception("bad rotation matrix size/type");
	}

	Mat rvec;
	if (R.size() == cv::Size(3, 3)) { cv::Rodrigues(R, rvec); }
	else { rvec = R; }

	ceres::AngleAxisToQuaternion<double>((double*)(rvec.data), data + Parameter::ROTATION);
}

void Camera::setTranslation(const Mat& t) {
	if ((t.type() != CV_64FC1) ||
		(t.size() != cv::Size(1, 3))) {

		throw ftl::exception("bad translation vector");
	}

	data[Parameter::TX] = t.at<double>(0);
	data[Parameter::TY] = t.at<double>(1);
	data[Parameter::TZ] = t.at<double>(2);
}


void Camera::setIntrinsic(const Mat& K, cv::Size sz) {
	size = sz;
	if ((K.type() != CV_64FC1) || (K.size() != cv::Size(3, 3))) {
		throw ftl::exception("bad intrinsic matrix");
	}

	data[Parameter::F] = K.at<double>(0, 0);
	data[Parameter::CX] = K.at<double>(0, 2);
	data[Parameter::CY] = K.at<double>(1, 2);
}

void Camera::setDistortion(const Mat& D) {
	if ((D.type() != CV_64FC1)) {
		throw ftl::exception("distortion coefficients must be CV_64FC1");
	}
	switch(D.total()) {
		case 12:
			/*
			data[Parameter::S1] = D.at<double>(8);
			data[Parameter::S2] = D.at<double>(9);
			data[Parameter::S3] = D.at<double>(10);
			data[Parameter::S4] = D.at<double>(11);
			*/
			[[fallthrough]];

		case 8:
			data[Parameter::K4] = D.at<double>(5);
			data[Parameter::K5] = D.at<double>(6);
			data[Parameter::K6] = D.at<double>(7);
			[[fallthrough]];

		case 5:
			data[Parameter::K3] = D.at<double>(4);
			[[fallthrough]];

		default:
			data[Parameter::K1] = D.at<double>(0);
			data[Parameter::K2] = D.at<double>(1);
			data[Parameter::P1] = D.at<double>(2);
			data[Parameter::P2] = D.at<double>(3);
	}
}

Camera::Camera(const Mat &K, const Mat &D, const Mat &R, const Mat &tvec, cv::Size sz) {
	setIntrinsic(K, D, sz);
	if (!R.empty()) { setRotation(R); }
	if (!tvec.empty()) { setTranslation(tvec); }
}

Camera::Camera(const ftl::calibration::CalibrationData::Calibration& calib) {
	setIntrinsic(calib.intrinsic.matrix(), calib.intrinsic.distCoeffs.Mat(), calib.intrinsic.resolution);
	setExtrinsic(calib.extrinsic.matrix()(cv::Rect(0, 0, 3, 3)), cv::Mat(calib.extrinsic.tvec));
}

ftl::calibration::CalibrationData::Intrinsic Camera::intrinsic() const {
	return ftl::calibration::CalibrationData::Intrinsic(intrinsicMatrix(), distortionCoefficients(), size);
}
ftl::calibration::CalibrationData::Extrinsic Camera::extrinsic() const {
	return ftl::calibration::CalibrationData::Extrinsic(rvec(), tvec());
}

Mat Camera::intrinsicMatrix() const {
	Mat K = Mat::eye(3, 3, CV_64FC1);
	K.at<double>(0, 0) = data[Parameter::F];
	K.at<double>(1, 1) = data[Parameter::F];
	K.at<double>(0, 2) = data[Parameter::CX];
	K.at<double>(1, 2) = data[Parameter::CY];
	return K;
}

Mat Camera::distortionCoefficients() const {
	Mat D;
	if      (Camera::n_distortion_parameters <= 4)  { D = Mat::zeros(1, 4, CV_64FC1); }
	else if (Camera::n_distortion_parameters <= 5)  { D = Mat::zeros(1, 5, CV_64FC1); }
	else if (Camera::n_distortion_parameters <= 8)  { D = Mat::zeros(1, 8, CV_64FC1); }
	else if (Camera::n_distortion_parameters <= 12) { D = Mat::zeros(1, 12, CV_64FC1); }
	else if (Camera::n_distortion_parameters <= 14) { D = Mat::zeros(1, 14, CV_64FC1); }

	switch(Camera::n_distortion_parameters) {
		case 14: // not used in OpenCV?
		case 12:
		case 8:
			D.at<double>(5) = data[Parameter::K4];
			D.at<double>(6) = data[Parameter::K5];
			D.at<double>(7) = data[Parameter::K6];
		case 5:
			D.at<double>(4) = data[Parameter::K3];
		case 4:
			D.at<double>(0) = data[Parameter::K1];
			D.at<double>(1) = data[Parameter::K2];
			D.at<double>(2) = data[Parameter::P1];
			D.at<double>(3) = data[Parameter::P2];
	}

	return D;
}

Mat Camera::rvec() const {
	cv::Mat rvec(cv::Size(3, 1), CV_64FC1);
	ceres::QuaternionToAngleAxis(data + Parameter::ROTATION,
		(double*)(rvec.data));
	return rvec;
}

Mat Camera::tvec() const {
	return Mat(Vec3d(data[Parameter::TX], data[Parameter::TY], data[Parameter::TZ]));
}

Mat Camera::rmat() const {
	Mat R;
	cv::Rodrigues(rvec(), R);
	return R;
}

Mat Camera::extrinsicMatrix() const {
	Mat T = Mat::eye(4, 4, CV_64FC1);
	rmat().copyTo(T(Rect(0, 0, 3, 3)));
	tvec().copyTo(T(Rect(3, 0, 1, 3)));
	return T;
}

Mat Camera::extrinsicMatrixInverse() const {
	return transform::inverse(extrinsicMatrix());
}

////////////////////////////////////////////////////////////////////////////////

struct ReprojectionError {
	/**
	 * Reprojection error.
	 *
	 * Camera model has _CAMERA_PARAMETERS parameters:
	 *
	 * - rotation and translation: q1, q2, q3, q4, tx, ty, tx
	 * - focal length: f (fx == fy assumed)
	 * - pricipal point: cx, cy
	 * - distortion coefficients: k1, k2, k3, k4, k5, k6, p1, p2
	 *
	 * Camera model documented in
	 * https://docs.opencv.org/master/d9/d0c/group__calib3d.html
	 * https://github.com/opencv/opencv/blob/b698d0a6ee12342a87b8ad739d908fd8d7ff1fca/modules/calib3d/src/calibration.cpp#L774
	 */
	explicit ReprojectionError(double observed_x, double observed_y)
		: observed_x(observed_x), observed_y(observed_y) {}

	template <typename T>
	bool operator()(const T* const camera,
					const T* const point,
					T* residuals) const {

		T p[3];
		ceres::QuaternionRotatePoint(camera + Camera::Parameter::ROTATION, point, p);


		p[0] += camera[Camera::Parameter::TX];
		p[1] += camera[Camera::Parameter::TY];
		p[2] += camera[Camera::Parameter::TZ];

		T x = T(p[0]) / p[2];
		T y = T(p[1]) / p[2];

		// Intrinsic parameters
		const T& f = camera[Camera::Parameter::F];
		const T& cx = camera[Camera::Parameter::CX];
		const T& cy = camera[Camera::Parameter::CY];

		// Distortion parameters
		const T& k1 = camera[Camera::Parameter::K1];
		const T& k2 = camera[Camera::Parameter::K2];
		const T& k3 = camera[Camera::Parameter::K3];
		const T& k4 = camera[Camera::Parameter::K4];
		const T& k5 = camera[Camera::Parameter::K5];
		const T& k6 = camera[Camera::Parameter::K6];
		const T& p1 = camera[Camera::Parameter::P1];
		const T& p2 = camera[Camera::Parameter::P2];

		const T r2 = x*x + y*y;
		const T r4 = r2*r2;
		const T r6 = r4*r2;

		// Radial distortion
		const T cdist = T(1.0) + k1*r2 + k2*r4 + k3*r6;
		// Radial distortion: rational model
		const T icdist = T(1.0)/(T(1.0) + k4*r2 + k5*r4 + k6*r6);
		// Tangential distortion
		const T pdistx =      (T(2.0)*x*y)*p1 + (r2 + T(2.0)*x*x)*p2;
		const T pdisty = (r2 + T(2.0)*y*y)*p1 +      (T(2.0)*x*y)*p2;
		// Apply distortion
		const T xd = x*cdist*icdist + pdistx;
		const T yd = y*cdist*icdist + pdisty;
		// Projected point position
		T predicted_x = f*xd + cx;
		T predicted_y = f*yd + cy;

		// Error: the difference between the predicted and observed position
		residuals[0] = predicted_x - T(observed_x);
		residuals[1] = predicted_y - T(observed_y);

		return true;
	}

	static ceres::CostFunction* Create(	const double observed_x,
										const double observed_y) {
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, Camera::n_parameters, 3>(
					new ReprojectionError(observed_x, observed_y)));
	}

	static ceres::CostFunction* Create(	const Point2d &observed) {
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, Camera::n_parameters, 3>(
					new ReprojectionError(observed.x, observed.y)));
	}

	double observed_x;
	double observed_y;
};

static ReprojectionError reproject_ = ReprojectionError(0.0, 0.0);

cv::Point2d ftl::calibration::projectPoint(const Camera& camera, const cv::Point3d& point) {
	cv::Point2d out;
	reproject_(static_cast<const double* const>(camera.data), reinterpret_cast<const double* const>(&(point.x)), reinterpret_cast<double*>(&(out.x)));
	return out;
}

////////////////////////////////////////////////////////////////////////////////

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
	if (points_.size() != 0) { throw ftl::exception("cameras can't be added after points"); }

	cameras_.push_back(&camera);
}

void BundleAdjustment::addCameras(vector<Camera>& cameras) {
	for (auto& camera : cameras) { addCamera(camera); }
}

void BundleAdjustment::addPoint(const vector<bool>& visibility, const vector<Point2d>& observations, Point3d& point) {
	if ((observations.size() != visibility.size()) ||
		(visibility.size() != cameras_.size())) { throw ftl::exception("observation and visibility vector sizes do not match"); }

	points_.push_back(BundleAdjustment::Point{visibility, observations, point});
}

void BundleAdjustment::addPoints(const vector<vector<bool>>& visibility, const vector<vector<Point2d>>& observations, vector<Point3d>& points) {
	if (observations.size() != visibility.size()) { throw ftl::exception("observation and visibility vector sizes do not match"); }

	auto npoints = points.size();
	auto ncameras = observations.size();

	for (unsigned c = 0; c < ncameras; c++) {
		if ((npoints != observations[c].size()) ||
			(npoints != visibility[c].size())) {
				throw ftl::exception("wrong number of points");
			}
	}

	vector<bool> add_vis;
	vector<Point2d> add_obs;
	for (size_t i = 0; i < npoints; i++) {
		add_obs.clear();
		add_vis.clear();
		for (size_t c = 0; c < ncameras; c++) {
			add_vis.push_back(visibility[c][i]);
			add_obs.push_back(observations[c][i]);
		}
		addPoint(add_vis, add_obs, points[i]);
	}
}

void BundleAdjustment::addPoint(const vector<Point2d>& observations, Point3d& point) {
	vector<bool> visibility(observations.size(), true);
	addPoint(visibility, observations, point);
}

void BundleAdjustment::addPoints(const vector<vector<Point2d>>& observations, std::vector<Point3d>& points) {
	if (observations.size() != points.size()) { throw ftl::exception("observation and visibility vector sizes do not match"); }
	for (size_t i = 0; i < points.size(); i++) {
		addPoint(observations[i], points[i]);
	}
}

void BundleAdjustment::addObject(const vector<Point3d> &object_points) {
	if (points_.size() % object_points.size() != 0) { throw ftl::exception("object does match point count"); }
	objects_.push_back(BundleAdjustment::Object {0, (int) points_.size(), object_points});
}

void BundleAdjustment::_setCameraParametrization(ceres::Problem &problem, const BundleAdjustment::Options &options) {

	vector<int> constant_camera_parameters;

	// apply options
	for (size_t i = 0; i < cameras_.size(); i++) {
		if (!options.rational_model) {
			cameras_[i]->data[Camera::Parameter::K4] = 0.0;
			cameras_[i]->data[Camera::Parameter::K5] = 0.0;
			cameras_[i]->data[Camera::Parameter::K6] = 0.0;
		}
		if (options.zero_distortion) {
			cameras_[i]->data[Camera::Parameter::K1] = 0.0;
			cameras_[i]->data[Camera::Parameter::K2] = 0.0;
			cameras_[i]->data[Camera::Parameter::K3] = 0.0;
			cameras_[i]->data[Camera::Parameter::K4] = 0.0;
			cameras_[i]->data[Camera::Parameter::K5] = 0.0;
			cameras_[i]->data[Camera::Parameter::K6] = 0.0;
			cameras_[i]->data[Camera::Parameter::P1] = 0.0;
			cameras_[i]->data[Camera::Parameter::P2] = 0.0;
		}
	}

	// set extrinsic paramters constant for all cameras
	if (!options.optimize_motion) {
		constant_camera_parameters.push_back(Camera::Parameter::Q1);
		constant_camera_parameters.push_back(Camera::Parameter::Q2);
		constant_camera_parameters.push_back(Camera::Parameter::Q3);
		constant_camera_parameters.push_back(Camera::Parameter::Q4);
		constant_camera_parameters.push_back(Camera::Parameter::TX);
		constant_camera_parameters.push_back(Camera::Parameter::TY);
		constant_camera_parameters.push_back(Camera::Parameter::TZ);
	}

	// set intrinsic parameters constant for all cameras
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
		constant_camera_parameters.push_back(Camera::Parameter::K4);
		constant_camera_parameters.push_back(Camera::Parameter::K5);
		constant_camera_parameters.push_back(Camera::Parameter::K6);
		constant_camera_parameters.push_back(Camera::Parameter::P1);
		constant_camera_parameters.push_back(Camera::Parameter::P2);
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
			std::unordered_set<int> constant_parameters(
				constant_camera_parameters.begin(),
				constant_camera_parameters.end());

			if (fix_extrinsic.find(i) != fix_extrinsic.end()) {
				constant_parameters.insert({
					Camera::Parameter::Q1, Camera::Parameter::Q2,
					Camera::Parameter::Q3, Camera::Parameter::Q4,
					Camera::Parameter::TX, Camera::Parameter::TY,
					Camera::Parameter::TZ
				});
			}

			if (fix_intrinsic.find(i) != fix_intrinsic.end()) {
				constant_parameters.insert({
					Camera::Parameter::F, Camera::Parameter::CX,
					Camera::Parameter::CY
				});
			}

			vector<int> params(constant_parameters.begin(), constant_parameters.end());

			if (params.size() == Camera::n_parameters) {
				// Ceres crashes if all parameters are set constant with
				// SubsetParameterization()
				// https://github.com/ceres-solver/ceres-solver/issues/347
				// https://github.com/ceres-solver/ceres-solver/commit/27183d661ecae246dbce6d03cacf84f39fba1f1e
				problem.SetParameterBlockConstant(getCameraPtr(i));
			}
			else if (params.size() > 0) {
				ceres::LocalParameterization* camera_parameterization = nullptr;

				if (constant_parameters.count(Camera::Parameter::ROTATION) == 0) {
					// quaternion parametrization
					for (auto& v : params) { v -= 4; }
					camera_parameterization =
						new ceres::ProductParameterization(
							new ceres::QuaternionParameterization(),
							new ceres::SubsetParameterization(Camera::n_parameters - 4, params));
				}
				else {
					// extrinsic parameters constant
					CHECK(constant_parameters.count(Camera::Parameter::Q1));
					CHECK(constant_parameters.count(Camera::Parameter::Q2));
					CHECK(constant_parameters.count(Camera::Parameter::Q3));
					CHECK(constant_parameters.count(Camera::Parameter::Q4));
					CHECK(constant_parameters.count(Camera::Parameter::TX));
					CHECK(constant_parameters.count(Camera::Parameter::TY));
					CHECK(constant_parameters.count(Camera::Parameter::TZ));

					camera_parameterization =
						new ceres::SubsetParameterization(Camera::n_parameters, params);
				}

				problem.SetParameterization(getCameraPtr(i), camera_parameterization);
			}
		}
	}
}

void BundleAdjustment::_buildBundleAdjustmentProblem(ceres::Problem &problem, const BundleAdjustment::Options &options) {

	ceres::LossFunction *loss_function = nullptr; // squared

	if (options.loss == Options::Loss::HUBER) {
		loss_function = new ceres::HuberLoss(1.0);
	}
	else if (options.loss == Options::Loss::CAUCHY) {
		loss_function = new ceres::CauchyLoss(1.0);
	}

	for (auto& point : points_) {
		for (size_t i = 0; i < point.observations.size(); i++) {
			if (!point.visibility[i]) { continue; }
			ceres::CostFunction* cost_function =
				ReprojectionError::Create(point.observations[i]);

			problem.AddResidualBlock(cost_function,
						loss_function,
						getCameraPtr(i),
						&(point.point.x)
			);
		}
	}

	_setCameraParametrization(problem, options);

	if (!options.optmize_structure) {
		// do not optimize points
		for (auto &point : points_) {
			problem.SetParameterBlockConstant(&(point.point.x));
		}
	}
}

void BundleAdjustment::_buildProblem(ceres::Problem &problem, const BundleAdjustment::Options &options) {

	_buildBundleAdjustmentProblem(problem, options);
}

void BundleAdjustment::run(const BundleAdjustment::Options &bundle_adjustment_options) {
	ceres::Problem problem;
	_buildProblem(problem, bundle_adjustment_options);

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
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

void BundleAdjustment::_reprojectionErrorSE(const int camera, double &error, double &npoints) const {
	error = 0.0;
	npoints = 0.0;

	for (const auto& point : points_) {
		if (!point.visibility[camera]) { continue; }
		const auto& obs = point.observations[camera];
		const auto& proj = projectPoint(*(cameras_[camera]), point.point);
		error += pow(proj.x - obs.x, 2);
		error += pow(proj.y - obs.y, 2);
		npoints += 1.0;
	}
}

double BundleAdjustment::reprojectionError(const int camera) const {
	double error, npoints;
	_reprojectionErrorSE(camera, error, npoints);
	return sqrt(error / npoints);
}

double BundleAdjustment::reprojectionError() const {
	double error = 0.0;
	double npoints = 0.0;
	for (size_t i = 0; i < cameras_.size(); i++) {
		double e, n;
		_reprojectionErrorSE(i, e, n);
		error += e;
		npoints += n;
	}
	return sqrt(error / npoints);
}
