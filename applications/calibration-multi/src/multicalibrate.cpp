#include "multicalibrate.hpp"

#include "calibration_data.hpp"
#include "calibration.hpp"
#include "optimization.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <loguru.hpp>

#include <map>

using cv::Mat;
using cv::Size;
using cv::Point2d;
using cv::Point3d;
using cv::Vec4d;
using cv::Scalar;

using std::string;
using std::vector;
using std::map;
using std::pair;
using std::make_pair;

double CalibrationTarget::estimateScale(vector<Point3d> points) {
	
	// 1. calculate statistics 
	// 2. reject possible outliers 
	// 3. calculate scale factor

	double f = 0.0;
	double S = 0.0;
	double m = 0.0;
	
	vector<double> d(points.size() / 2, 0.0);

	for (size_t i = 0; i < points.size(); i += 2) {
		const Point3d &p1 = points[i];
		const Point3d &p2 = points[i + 1];

		Point3d p = p1 - p2;

		double x = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
		double prev_mean = m;
		d[i/2] = x;

		f = f + 1.0;
		m = m + (x - m) / f;
		S = S + (x - m) * (x - prev_mean);

	}

	double stddev = sqrt(S / f);
	f = 0.0;

	int outliers = 0;
	double scale = 0.0;

	for (double l : d) {
		// TODO:	* Parameterize how large deviation allowed
		//			* Validate this actually improves quality

		if (abs(l - m) > 3.0 * stddev) {
			outliers++;
		}
		else {
			f += 1.0;
			scale += 1.0 / l;
		}
		DCHECK(scale != INFINITY);
	}

	if (outliers != 0) {
		LOG(WARNING) << "Outliers (large std. deviation in scale): " << outliers;
	}

	LOG(INFO) << "calibration target std. dev. " <<  stddev << " (" << (int) f << " samples), scale: " << scale * calibration_bar_length_ / f;

	return scale * calibration_bar_length_ / f;

	// TODO:	LM-optimization for scale.
}

MultiCameraCalibrationNew::MultiCameraCalibrationNew(
			size_t n_cameras, size_t reference_camera, Size resolution,
			CalibrationTarget target, int fix_intrinsics) :
		
	target_(target),
	visibility_graph_(n_cameras),
	is_calibrated_(false),
	n_cameras_(n_cameras),
	reference_camera_(reference_camera),
	min_visible_points_(50),

	fix_intrinsics_(fix_intrinsics == 1 ? 5 : 0),
	resolution_(resolution),
	K_(n_cameras),
	dist_coeffs_(n_cameras),
	R_(n_cameras),
	t_(n_cameras),

	points3d_optimized_(n_cameras),
	points3d_(n_cameras),
	points2d_(n_cameras),
	visible_(n_cameras),

	fm_method_(cv::FM_8POINT), // RANSAC/LMEDS results need validation (does not work)
	fm_ransac_threshold_(0.95),
	fm_confidence_(0.90)
{
	for (auto &K : K_) { K = Mat::eye(Size(3, 3), CV_64FC1); }
	for (auto &d : dist_coeffs_) { d = Mat(Size(5, 1), CV_64FC1, Scalar(0.0)); }
}

Mat MultiCameraCalibrationNew::getCameraMat(size_t idx) {
	DCHECK(idx < n_cameras_);
	Mat K;
	K_[idx].copyTo(K);
	return K;
}


Mat MultiCameraCalibrationNew::getCameraMatNormalized(size_t idx, double scale_x, double scale_y)
{
	Mat K = getCameraMat(idx);
	CHECK((scale_x != 0.0 && scale_y != 0.0) || ((scale_x == 0.0) && scale_y == 0.0));

	scale_x = scale_x / (double) resolution_.width;
	scale_y = scale_y / (double) resolution_.height;

	Mat scale(Size(3, 3), CV_64F, 0.0);
	scale.at<double>(0, 0) = scale_x;
	scale.at<double>(1, 1) = scale_y;
	scale.at<double>(2, 2) = 1.0;
	
	return (scale * K);
}

Mat MultiCameraCalibrationNew::getDistCoeffs(size_t idx) {
	DCHECK(idx < n_cameras_);
	Mat D;
	dist_coeffs_[idx].copyTo(D);
	return D;
}

void MultiCameraCalibrationNew::setCameraParameters(size_t idx, const Mat &K, const Mat &distCoeffs) {
	CHECK(idx < n_cameras_);
	CHECK(K.size() == Size(3, 3));
	CHECK(distCoeffs.total() == 5);
	K.convertTo(K_[idx], CV_64FC1);
	distCoeffs.convertTo(dist_coeffs_[idx], CV_64FC1);
}

void MultiCameraCalibrationNew::setCameraParameters(size_t idx, const Mat &K) {
	DCHECK(idx < n_cameras_);
	setCameraParameters(idx, K, dist_coeffs_[idx]);
}

void MultiCameraCalibrationNew::addPoints(vector<vector<Point2d>> points, vector<int> visible) {
	DCHECK(points.size() == visible.size());
	DCHECK(visible.size() == n_cameras_);

	for (size_t i = 0; i < n_cameras_; i++) {
		visible_[i].insert(visible_[i].end(), points[i].size(), visible[i]);
		points2d_[i].insert(points2d_[i].end(), points[i].begin(), points[i].end());
	}
	visibility_graph_.update(visible);
}

void MultiCameraCalibrationNew::reset() {
	is_calibrated_ = false;
	weights_ = vector(n_cameras_, vector(points2d_[0].size(), 0.0));
	inlier_ = vector(n_cameras_, vector(points2d_[0].size(), 0));
	points3d_ = vector(n_cameras_, vector(points2d_[0].size(), Point3d()));
	points3d_optimized_ = vector(points2d_[0].size(), Point3d());
	R_ = vector<Mat>(n_cameras_, Mat::eye(Size(3, 3), CV_64FC1));
	t_ = vector<Mat>(n_cameras_, Mat(Size(1, 3), CV_64FC1, Scalar(0.0)));
}

void MultiCameraCalibrationNew::saveInput(const string &filename) {
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	saveInput(fs);
	fs.release();
}

void MultiCameraCalibrationNew::saveInput(cv::FileStorage &fs) {
	fs << "resolution" << resolution_;
	fs << "K" << K_;
	fs << "D" << dist_coeffs_;
	fs << "points2d" << points2d_;
	fs << "visible" << visible_;
}

void MultiCameraCalibrationNew::loadInput(const std::string &filename, const vector<size_t> &cameras_in) {
	points2d_.clear();
	points3d_.clear();
	points3d_optimized_.clear();
	visible_.clear();
	inlier_.clear();
	K_.clear();
	dist_coeffs_.clear();
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	vector<Mat> K;
	vector<vector<Point2d>> points2d;
	vector<vector<int>> visible;
	fs["K"] >> K;
	fs["D"] >> dist_coeffs_;
	fs["points2d"] >> points2d;
	fs["visible"] >> visible;
	fs["resolution"] >> resolution_;
	fs.release();
	
	vector<size_t> cameras;
	if (cameras_in.size() == 0) {
		cameras.resize(K.size());
		size_t i = 0;
		for (auto &c : cameras) { c = i++; }
	} 
	else {
		cameras.reserve(cameras_in.size());
		for (auto &c : cameras_in) { cameras.push_back(c); }
	}
	
	n_cameras_ = cameras.size();

	points2d_.resize(n_cameras_);
	points3d_.resize(n_cameras_);
	visible_.resize(n_cameras_);

	for (auto const &c : cameras) {
		K_.push_back(K[c]);
		LOG(INFO) << K[c];
	}
	for (size_t c = 0; c < n_cameras_; c++) {
		points2d_[c].reserve(visible[0].size());
		points3d_[c].reserve(visible[0].size());
		visible_[c].reserve(visible[0].size());
		points3d_optimized_.reserve(visible[0].size());
	}

	visibility_graph_ = Visibility(n_cameras_);
	dist_coeffs_.resize(n_cameras_);

	vector<vector<Point2d>> points2d_add(n_cameras_, vector<Point2d>());
	vector<int> visible_add(n_cameras_);
	for (size_t i = 0; i < visible[0].size(); i += target_.n_points) {
		int count = 0;
		for (size_t c = 0; c < n_cameras_; c++) {
			count += visible[c][i];
			points2d_add[c].clear();
			points2d_add[c].insert(
								points2d_add[c].begin(),
								points2d[cameras[c]].begin() + i,
								points2d[cameras[c]].begin() + i + target_.n_points);
			visible_add[c] = visible[cameras[c]][i];
		}
		if (count >= 2) {
			addPoints(points2d_add, visible_add);
		}
	}
	reset();
	
	DCHECK(points2d_.size() == n_cameras_);
	DCHECK(points2d_.size() == visible_.size());
	size_t len = visible_[0].size();
	for (size_t i = 0; i < n_cameras_; i++) {
		DCHECK(visible_[i].size() == len);
		DCHECK(points2d_[i].size() == visible_[i].size());
	}
}

size_t MultiCameraCalibrationNew::getViewsCount() {
	return points2d_[0].size() / target_.n_points;
}

size_t MultiCameraCalibrationNew::getOptimalReferenceCamera() {
	return (size_t) visibility_graph_.getOptimalCamera();
}

bool MultiCameraCalibrationNew::isVisible(size_t camera, size_t idx) {
	return visible_[camera][idx] == 1;
}

bool MultiCameraCalibrationNew::isValid(size_t camera, size_t idx) {
	return inlier_[camera][idx] >= 0;
}

bool MultiCameraCalibrationNew::isValid(size_t idx) {
	for (auto camera : inlier_) {
		if (camera[idx] > 0) return true;
	}
	return false;
}

vector<Point2d> MultiCameraCalibrationNew::getPoints(size_t camera, size_t idx) {
	return vector<Point2d> (points2d_[camera].begin() + idx * (target_.n_points), 
							points2d_[camera].begin() + idx * (target_.n_points + 1));
}


void MultiCameraCalibrationNew::updatePoints3D(size_t camera, Point3d new_point,
		size_t idx, const Mat &R, const Mat &t) {
	
	int &f = inlier_[camera][idx];
	Point3d &point = points3d_[camera][idx];
	new_point = transformPoint(new_point, R, t);

	if (f == -1) return;

	if (f > 0) {
		// TODO:	remove parameter (10.0 cm - 1.0m); over 0.25m difference
		//			would most likely suggest very bad triangulation (sync? wrong match?)
		// 			instead store all triangulations and handle outliers
		//			(perhaps inverse variance weighted mean?)
		
		if (euclideanDistance(point, new_point) > 10.0) {
			LOG(ERROR) << "bad value (skipping) " << "(" << point << " vs " << new_point << ")";
			f = -1;
		}
		else {
			point = (point * f + new_point) / (double) (f + 1);
			f++;
		}
	}
	else {
		point = new_point;
		f = 1;
	}
}

void MultiCameraCalibrationNew::updatePoints3D(size_t camera, vector<Point3d> points,
		vector<size_t> idx, const Mat &R, const Mat &t) {
	
	for (size_t i = 0; i < idx.size(); i++) {
		updatePoints3D(camera, points[i], idx[i], R, t);
	}
}

void MultiCameraCalibrationNew::getVisiblePoints(
		vector<size_t> cameras, vector<vector<Point2d>> &points, vector<size_t> &idx) {
	
	size_t n_points_total = points2d_[0].size();
	DCHECK(cameras.size() <= n_cameras_);
	DCHECK(n_points_total % target_.n_points == 0);
	
	idx.clear();
	idx.reserve(n_points_total);
	points.clear();
	points.resize(cameras.size(), {});
	
	for (size_t i = 0; i < n_points_total; i += target_.n_points) {
		bool visible_all = true;

		for (auto c : cameras) {
			for (size_t j = 0; j < target_.n_points; j++) {
				visible_all &= isVisible(c, i + j);
			}
		}
		
		if (!visible_all) { continue; }

		for (size_t j = 0; j < target_.n_points; j++) {
			idx.push_back(i + j);
		}

		for (size_t c = 0; c < cameras.size(); c++) {
			points[c].insert(points[c].end(),
							 points2d_[cameras[c]].begin() + i,
							 points2d_[cameras[c]].begin() + i + target_.n_points
			);
		}
	}

	for (auto p : points) {	DCHECK(idx.size() == p.size()); }
}

double MultiCameraCalibrationNew::calibratePair(size_t camera_from, size_t camera_to, Mat &rmat, Mat &tvec) {
	

	vector<size_t> idx;
	vector<Point2d> points1, points2;
	{
		vector<vector<Point2d>> points2d;
		getVisiblePoints({camera_from, camera_to}, points2d, idx);

		points1 = points2d[0];
		points2 = points2d[1];
	}
	DCHECK(points1.size() % target_.n_points == 0);
	DCHECK(points1.size() == points2.size());

	// cameras possibly lack line of sight?
	DCHECK(points1.size() > 8);
	
	Mat &K1 = K_[camera_from];
	Mat &K2 = K_[camera_to];
	/*
	vector<uchar> inliers;
	Mat F, E;
	F = cv::findFundamentalMat(points1, points2, fm_method_, fm_ransac_threshold_, fm_confidence_, inliers);

	if (F.empty())
	{
		LOG(ERROR) << "Fundamental matrix estimation failed. Possibly degenerate configuration?";
		return INFINITY;
	}

	E = K2.t() * F * K1;

	// Only include inliers
	if (fm_method_ == cv::FM_LMEDS || fm_method_ == cv::FM_RANSAC) {
		vector<Point2d> inliers1, inliers2;
		vector<size_t> inliers_idx;

		inliers1.reserve(points1.size());
		inliers2.reserve(points1.size());
		inliers_idx.reserve(points1.size());

		for (size_t i = 0; i < inliers.size(); i += target_.n_points) {
			bool inlier = true;
			
			for (size_t j = 0; j < target_.n_points; j++) {
				inlier &= inliers[i+j];
			}

			if (inlier) {
				inliers1.insert(inliers1.end(), points1.begin() + i, points1.begin() + i + target_.n_points);
				inliers2.insert(inliers2.end(), points2.begin() + i, points2.begin() + i + target_.n_points);
				inliers_idx.insert(inliers_idx.end(), idx.begin() + i, idx.begin() + i + target_.n_points);
			}
		}
		
		LOG(INFO) << "Total points: " << points1.size() << ", inliers: " << inliers1.size();
		double ratio_good_points = (double) inliers1.size() / (double) points1.size();
		if (ratio_good_points < 0.66) {
			// TODO: ... 
			LOG(WARNING) << "Over 1/3 of points rejected!";
			if (ratio_good_points < 0.33) { LOG(FATAL) << "Over 2/3 points rejected!"; }
		}
		
		DCHECK(inliers1.size() == inliers_idx.size());
		DCHECK(inliers2.size() == inliers_idx.size());

		std::swap(inliers1, points1);
		std::swap(inliers2, points2);
		std::swap(inliers_idx, idx);
	}
	
	// Estimate initial rotation matrix and translation vector and triangulate
	// points (in camera 1 coordinate system).

	Mat R1, R2, t1, t2;
	R1 = Mat::eye(Size(3, 3), CV_64FC1);
	t1 = Mat(Size(1, 3), CV_64FC1, Scalar(0.0));

	vector<Point3d> points3d;
	// Convert homogeneous coordinates 
	{
		Mat points3dh;
		recoverPose(E, points1, points2, K1, K2, R2, t2, 1000.0, points3dh);
		points3d.reserve(points3dh.cols);

		for (int col = 0; col < points3dh.cols; col++) {
			Point3d p = Point3d(points3dh.at<double>(0, col),
								points3dh.at<double>(1, col),
								points3dh.at<double>(2, col))
								/ points3dh.at<double>(3, col);
			points3d.push_back(p);
		}
	}
	DCHECK(points3d.size() == points1.size());

	// Estimate and apply scale factor
	{
		double scale = ftl::calibration::optimizeScale(object_points_, points3d);
		t1 = t1 * scale;
		t2 = t2 * scale;
	}

	// Reprojection error before BA
	{
		// SBA should report squared mean error
		const double err1 = reprojectionError(points3d, points1, K1, R1, t1);
		const double err2 = reprojectionError(points3d, points2, K2, R2, t2);
		
		if (abs(err1 - err2) > 2.0) {
			LOG(INFO) << "Initial reprojection error (camera " << camera_from << "): " << err1;
			LOG(INFO) << "Initial reprojection error (camera " << camera_to << "): " << err2;
		}
		LOG(INFO)	<< "Initial reprojection error (" << camera_from << ", " << camera_to << "): "
					<< sqrt(err1 * err1 + err2 * err2);
		
	}
	
	// Bundle Adjustment
	
	double err = INFINITY;
	{
		auto cameras = vector<ftl::calibration::Camera> {
			ftl::calibration::Camera(K1, dist_coeffs_[camera_from], R1, t1),
			ftl::calibration::Camera(K2, dist_coeffs_[camera_to], R2, t2)
		};

		ftl::calibration::BundleAdjustment ba;
		ba.addCameras(cameras);
		for (size_t i = 0; i < points3d.size(); i++) {
			ba.addPoint({points1[i], points2[i]}, points3d[i]);
		}

		ftl::calibration::BundleAdjustment::Options options;
		options.fix_camera_extrinsic = {0};
		options.optimize_intrinsic = false;

		ba.run(options);
		// TODO: ... 
		err = sqrt(ba.reprojectionError(0) * ba.reprojectionError(1));

		R2 = cameras[1].rmat();
		t2 = Mat(cameras[1].tvec());
	}

	calculateTransform(R2, t2, R1, t1, rmat, tvec);
	*/
	
	Mat R1, R2, t1, t2;
	R1 = Mat::eye(Size(3, 3), CV_64FC1);
	t1 = Mat(Size(1, 3), CV_64FC1, Scalar(0.0));
	
	vector<Point3d> points3d;
	
	double err = ftl::calibration::computeExtrinsicParameters(K1, dist_coeffs_[camera_from], K2, dist_coeffs_[camera_to], points1, points2, object_points_, R2, t2, points3d);
	calculateTransform(R2, t2, R1, t1, rmat, tvec);
	

	// Store and average 3D points for both cameras (skip garbage)
	if (err < 10.0) {
		Mat rmat1, tvec1;
		updatePoints3D(camera_from, points3d, idx, R1, t1);
		updatePoints3D(camera_to, points3d, idx, R2, t2);
	}
	else {
		LOG(ERROR)	<< "Large RMS error ("
					<< reprojectionError(points3d, points2, K2, rmat, tvec)
					<< "), not updating points!";
	}

	//LOG(INFO) << reprojectionError(points3d, points1, K1, R1, t1);
	//LOG(INFO) << reprojectionError(points3d, points2, K2, R2, t2);

	return err;
}

Point3d MultiCameraCalibrationNew::getPoint3D(size_t camera, size_t idx) {
	return points3d_[camera][idx];
}

void MultiCameraCalibrationNew::calculateMissingPoints3D() {
	points3d_optimized_.clear();
	points3d_optimized_.resize(points3d_[reference_camera_].size());

	for (size_t i = 0; i < points3d_optimized_.size(); i++) {
		if (inlier_[reference_camera_][i] > 0) {
			points3d_optimized_[i] = points3d_[reference_camera_][i];
			continue;
		}

		if (!isValid(i)) continue;

		double f = 0.0;
		Point3d point(0.0, 0.0, 0.0);
		for (size_t c = 0; c < n_cameras_; c++) {
			if (inlier_[c][i] <= 0) { continue; }
			point += transformPoint(getPoint3D(c, i), R_[c], t_[c]);
			f += 1.0;
		}

		DCHECK(f != 0.0);

		points3d_optimized_[i] = point / f;
	}
}

double MultiCameraCalibrationNew::getReprojectionError(size_t c_from, size_t c_to, const Mat &K, const Mat &R, const Mat &t) {

	vector<Point2d> points2d;
	vector<Point3d> points3d;

	for (size_t i = 0; i < points2d_[c_from].size(); i++) {
		if (!isValid(i) || !isVisible(c_from, i) || !isVisible(c_to, i)) continue;
		points2d.push_back(points2d_[c_from][i]);
		points3d.push_back(points3d_[c_to][i]);
	}

	return reprojectionError(points3d, points2d, K, R, t);
}

double MultiCameraCalibrationNew::getReprojectionErrorOptimized(size_t c_from, const Mat &K, const Mat &R, const Mat &t) {
	
	vector<Point2d> points2d;
	vector<Point3d> points3d;

	for (size_t i = 0; i < points2d_[c_from].size(); i++) {
		if (!isValid(i) || !isVisible(c_from, i)) continue;
		points2d.push_back(points2d_[c_from][i]);
		points3d.push_back(points3d_optimized_[i]);
	}

	return reprojectionError(points3d, points2d, K, R, t);
}


double MultiCameraCalibrationNew::calibrateAll(int reference_camera) {
	if (reference_camera != -1) {
		DCHECK(reference_camera >= 0 && reference_camera < n_cameras_);
		reference_camera_ = reference_camera; 
	}

	for (const auto &K : K_) {
		LOG(INFO) << K;
	}

	reset(); // remove all old calibration results
	map<pair<size_t, size_t>, pair<Mat, Mat>> transformations; 
	
	// All cameras should be calibrated pairwise; otherwise all possible 3D
	// points are not necessarily triangulated

	auto paths = visibility_graph_.findShortestPaths(reference_camera_);
	
	for (size_t c1 = 0; c1 < n_cameras_; c1++) {
	for (size_t c2 = c1; c2 < n_cameras_; c2++) {
		if (c1 == c2) {
			transformations[make_pair(c1, c2)] = 
				make_pair(Mat::eye(Size(3, 3), CV_64FC1),
				Mat(Size(1, 3), CV_64FC1, Scalar(0.0))
			);
			continue;
		}

		size_t n_visible = getVisiblePointsCount({c1, c2});

		if (n_visible < min_visible_points_) {
			LOG(INFO)	<< "Not enough (" << min_visible_points_ << ") points  between "
						<< "cameras " << c1 << " and " << c2 << " (" << n_visible << " points), "
						<< "skipping";
			continue;
		}
		LOG(INFO)	<< "Running pairwise calibration for cameras "
					<< c1 << " and " << c2 << "(" << n_visible << " points)";

		if (transformations.find(make_pair(c2, c1)) != transformations.end()) {
			continue;
		}
		Mat R, t, R_i, t_i;

		// TODO: threshold parameter, 16.0 possibly too high

		if (calibratePair(c1, c2, R, t) > 16.0) {
			LOG(ERROR)	<< "Pairwise calibration failed, skipping cameras "
						<< c1 << " and " << c2;
			visibility_graph_.deleteEdge(c1, c2);
			continue;
		}

		calculateInverse(R, t, R_i, t_i);

		transformations[make_pair(c2, c1)] = make_pair(R, t);
		transformations[make_pair(c1, c2)] = make_pair(R_i, t_i);
	}}

	for (size_t c = 0; c < paths.size(); c++) {
		Mat R_chain = Mat::eye(Size(3, 3), CV_64FC1);
		Mat t_chain = Mat(Size(1, 3), CV_64FC1, Scalar(0.0));
		LOG(INFO) << "Chain for camera " << c;
		for (auto e: paths[c]) {
			CHECK(transformations.find(e) != transformations.end()) << "chain not calculated; pairwise calibration possibly failed earlier?";
			LOG(INFO) << e.first << " -> " << e.second;
			Mat R = transformations[e].first;
			Mat t = transformations[e].second;
			R_chain = R * R_chain;
			t_chain = t + R * t_chain;
		}

		R_[c] = R_chain;
		t_[c] = t_chain;
		/*R_[c] = transformations[make_pair(reference_camera_, c)].first;
		t_[c] = transformations[make_pair(reference_camera_, c)].second;
		DCHECK(R_[c].size() == Size(3, 3));
		DCHECK(t_[c].size() == Size(1, 3));*/
	}
	
	calculateMissingPoints3D();
	
	for (size_t c_from = 0; c_from < n_cameras_; c_from++) {
		if (c_from == reference_camera_) continue;
		Mat R, t;
		calculateInverse(R_[c_from], t_[c_from], R, t);
		LOG(INFO)	<< "Error before BA, cameras " << reference_camera_ << " and " << c_from << ": "
					<< getReprojectionErrorOptimized(c_from, K_[c_from], R, t);
	
	}

	double err;
	{
		auto cameras = vector<ftl::calibration::Camera>();
		
		for (size_t i = 0; i < n_cameras_; i++) {
			calculateInverse(R_[i], t_[i], R_[i], t_[i]);
			cameras.push_back(ftl::calibration::Camera(K_[i], dist_coeffs_[i], R_[i], t_[i]));
		}

		ftl::calibration::BundleAdjustment ba;
		ba.addCameras(cameras);

		for (size_t i = 0; i < points3d_optimized_.size(); i++) {
			
			auto &p = points3d_optimized_[i];
			DCHECK(!isnanl(p.x) && !isnanl(p.y) && !isnanl(p.z));

			int count = 0;
			for (size_t c = 0; c < n_cameras_; c++) {
				if (isVisible(c, i) && isValid(c, i)) { count++; }
			}
			
			if (count < 2) continue;
			
			vector<bool> visible(n_cameras_);
			vector<Point2d> points2d(n_cameras_);

			for (size_t c = 0; c < n_cameras_; c++) {
				bool good = isVisible(c, i) && isValid(c, i);
				visible[c] = good;
				points2d[c] = points2d_[c][i];
			}

			ba.addPoint(visible, points2d, p);
		}

		ftl::calibration::BundleAdjustment::Options options;
		options.optimize_intrinsic = !fix_intrinsics_;
		options.fix_distortion = true;
		options.max_iter = 50;
		options.fix_camera_extrinsic = {reference_camera};
		options.verbose = true;
 
		err = ba.reprojectionError();
		ba.run(options);

		for (size_t i = 0; i < n_cameras_; i++) {
			R_[i] = cameras[i].rmat();
			t_[i] = cameras[i].tvec();
			K_[i] = cameras[i].intrinsicMatrix();
			//dist_coeffs_[i] = D; // not updated
			calculateInverse(R_[i], t_[i], R_[i], t_[i]);
		}
	}

	for (size_t c_from = 0; c_from < n_cameras_; c_from++) {
		if (c_from == reference_camera_) continue;
		Mat R, t;
		calculateInverse(R_[c_from], t_[c_from], R, t);
		LOG(INFO)	<< "Error (RMS) after BA, cameras " << reference_camera_ << " and " << c_from << ": "
					<< getReprojectionErrorOptimized(c_from, K_[c_from], R, t);
	
	}

	is_calibrated_ = true;
	return err;
}

void MultiCameraCalibrationNew::projectPointsOriginal(size_t camera_src, size_t camera_dst, size_t idx, vector<Point2d> &points) {
	
}

void MultiCameraCalibrationNew::projectPointsOptimized(size_t camera_dst, size_t idx, vector<Point2d> &points) {
	// TODO:	indexing does not match input (points may be skipped in loadInput())

	points.clear();
	size_t i = target_.n_points * idx;
	
	if (!isValid(i)) return;

	Point3d p1(points3d_optimized_[i]);
	Point3d p2(points3d_optimized_[i + 1]);

	if (!std::isfinite(p1.x) || !std::isfinite(p2.x)) {
		// DEBUG: should not happen
		LOG(ERROR) << "Bad point! (no valid triangulation)";
		return; 
	}
	
	Mat R, tvec, rvec;
	calculateTransform(R_[reference_camera_], t_[reference_camera_], R_[camera_dst], t_[camera_dst], R, tvec);
	
	cv::Rodrigues(R, rvec);
	cv::projectPoints(	vector<Point3d> { p1, p2 },
						rvec, tvec, K_[camera_dst], dist_coeffs_[camera_dst], points);
}

void MultiCameraCalibrationNew::getCalibration(vector<Mat> &R, vector<Mat> &t) {
	DCHECK(is_calibrated_);
	R.resize(n_cameras_);
	t.resize(n_cameras_);

	for (size_t i = 0; i < n_cameras_; i++) {
		R_[i].copyTo(R[i]);
		t_[i].copyTo(t[i]);
	}
}