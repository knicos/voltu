#pragma once

#include <opencv2/core.hpp>

#include "visibility.hpp"
#include "util.hpp"

using cv::Mat;
using cv::Size;
using cv::Point2d;
using cv::Point3d;
using cv::Vec4d;
using cv::Scalar;

using std::vector;
using std::pair;

class CalibrationTarget {
public:
	CalibrationTarget(double length) :
		n_points(2),
		calibration_bar_length_(length)
	{}
	/* @brief	Estimate scale factor.
	 * @param	3D points (can pass n views)
	 */
	double estimateScale(vector<Point3d> points3d);
	size_t n_points;

private:
	double calibration_bar_length_;
};

class MultiCameraCalibrationNew {
public:
	MultiCameraCalibrationNew(	size_t n_cameras, size_t reference_camera,
								CalibrationTarget target, int fix_intrinsics=1);
	
	void setCameraParameters(size_t idx, const Mat &K, const Mat &distCoeffs);
	void setCameraParameters(size_t idx, const Mat &K);

	void addPoints(vector<vector<Point2d>> points2d, vector<int> visibility);

	size_t getViewsCount();
	size_t getCamerasCount() { return n_cameras_; }
	size_t getOptimalReferenceCamera();

	size_t getMinVisibility() { return visibility_graph_.getMinVisibility(); }
	size_t getViewsCount(size_t camera) { return visibility_graph_.getViewsCount(camera); }

	void setFixIntrinsic(int value) { fix_intrinsics_ = (value == 1 ? 5 : 0); }

	void loadInput(const std::string &filename, const vector<size_t> &cameras = {});

	void saveInput(cv::FileStorage &fs);
	void saveInput(const std::string &filename);

	Mat getCameraMat(size_t idx);
	Mat getDistCoeffs(size_t idx);

	double calibrateAll(int reference_camera = -1);
	double getReprojectionError();
	void getCalibration(vector<Mat> &R, vector<Mat> &t);

	void projectPointsOriginal(size_t camera_src, size_t camera_dst, size_t idx, vector<Point2d> &points);
	void projectPointsOptimized(size_t camera_dst, size_t idx, vector<Point2d> &points);

protected:
	bool isVisible(size_t camera, size_t idx);
	bool isValid(size_t camera, size_t idx);
	bool isValid(size_t idx);

	Point3d getPoint3D(size_t camera, size_t i);

	vector<Point2d> getPoints(size_t camera, size_t idx);
	vector<vector<Point2d>> getAllPoints(size_t camera, vector<size_t> idx);

	void getVisiblePoints(	vector<size_t> cameras,
							vector<vector<Point2d>> &points,
							vector<size_t> &idx);

	size_t getVisiblePointsCount(vector<size_t> cameras) {
		// TODO: for pairs can use visibility graph adjacency matrix
		vector<vector<Point2d>> points2d;
		vector<size_t> idx;
		getVisiblePoints(cameras, points2d, idx);
		return idx.size();
	}

	size_t getTotalPointsCount() {
		return points2d_[0].size();
	}

	vector<Point3d> getPoints3D(size_t idx);

	/* @brief	Find points which are visible on all cameras. Returns
	 * 			corresponding indices in idx vector.
	 */
	void getVisiblePoints3D(vector<size_t> cameras,
							vector<vector<Point3d>> &points,
							vector<size_t> &idx);

	/* @brief	Update 3D points with new values. If no earlier data, new data
	 *			is used as is, otherwise calculates average.
	 */
	void updatePoints3D(size_t camera, Point3d new_point, size_t idx, const Mat &R, const Mat &t);
	void updatePoints3D(size_t camera, vector<Point3d> new_points, vector<size_t> idx, const Mat &R, const Mat &t);

	/* @brief	Calculates 3D points that are not visible in reference camera
	 *			from transformations in visible cameras.
	 */
	void calculateMissingPoints3D();

	void getTransformation(size_t camera_from, size_t camera_to, Mat &R, Mat &T);
	double calibratePair(size_t camera_from, size_t camera_to, Mat &R, Mat &T);

	/* @brief	Calculate reprojection error of visible points (triangulation) */
	double getReprojectionError(size_t c_from, size_t c_to, const Mat &K, const Mat &R, const Mat &T);

	/* @brief	Calculate reprojection error of visible points (optimized/averaged points) */
	double getReprojectionErrorOptimized(size_t c_from, const Mat &K, const Mat &R, const Mat &T);

	/* @brief	Remove old calibration data calculated by calibrateAll */
	void reset();

private:
	CalibrationTarget target_;
	Visibility visibility_graph_; 

	bool is_calibrated_;
	size_t n_cameras_;
	size_t reference_camera_;
	size_t min_visible_points_;
	int fix_intrinsics_;

	vector<Mat> K_;
	vector<Mat> dist_coeffs_;
	vector<Mat> R_;
	vector<Mat> t_;

	vector<Point3d> points3d_optimized_;
	vector<vector<Point3d>> points3d_;
	vector<vector<Point2d>> points2d_;
	vector<vector<int>> visible_;
	vector<vector<int>> inlier_; // "inlier"

	int fm_method_;
	double fm_ransac_threshold_;
	double fm_confidence_;
};
