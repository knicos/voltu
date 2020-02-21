#include "calibration.hpp"
#include "ftl/calibration/optimize.hpp"

#include "loguru.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

using std::vector;

using cv::Mat;
using cv::Size;
using cv::Point2d;
using cv::Point3d;
using cv::Vec3d;

using cv::norm;

using ftl::calibration::BundleAdjustment;

using namespace ftl::calibration;

int ftl::calibration::recoverPose(const Mat &E, const vector<Point2d> &_points1,
	const vector<Point2d> &_points2, const Mat &_cameraMatrix1,
	const Mat &_cameraMatrix2, Mat &_R, Mat &_t, double distanceThresh,
	Mat &triangulatedPoints) {

	Mat cameraMatrix1;
	Mat cameraMatrix2;
	Mat cameraMatrix;

	Mat points1(_points1.size(), 2, CV_64FC1);
	Mat points2(_points2.size(), 2, CV_64FC1);

	CHECK(points1.size() == points2.size());

	for (size_t i = 0; i < _points1.size(); i++) {
		auto p1 = points1.ptr<double>(i);
		p1[0] = _points1[i].x;
		p1[1] = _points1[i].y;

		auto p2 = points2.ptr<double>(i);
		p2[0] = _points2[i].x;
		p2[1] = _points2[i].y;
	}

	_cameraMatrix1.convertTo(cameraMatrix1, CV_64F);
	_cameraMatrix2.convertTo(cameraMatrix2, CV_64F);
	cameraMatrix = Mat::eye(Size(3, 3), CV_64FC1);

	double fx1 = cameraMatrix1.at<double>(0,0);
	double fy1 = cameraMatrix1.at<double>(1,1);
	double cx1 = cameraMatrix1.at<double>(0,2);
	double cy1 = cameraMatrix1.at<double>(1,2);

	double fx2 = cameraMatrix2.at<double>(0,0);
	double fy2 = cameraMatrix2.at<double>(1,1);
	double cx2 = cameraMatrix2.at<double>(0,2);
	double cy2 = cameraMatrix2.at<double>(1,2);

	points1.col(0) = (points1.col(0) - cx1) / fx1;
	points1.col(1) = (points1.col(1) - cy1) / fy1;

	points2.col(0) = (points2.col(0) - cx2) / fx2;
	points2.col(1) = (points2.col(1) - cy2) / fy2;

	// TODO mask
	// cameraMatrix = I (for details, see OpenCV's recoverPose() source code)
	// modules/calib3d/src/five-point.cpp (461)
	//
	// https://github.com/opencv/opencv/blob/371bba8f54560b374fbcd47e7e02f015ac4969ad/modules/calib3d/src/five-point.cpp#L461

	return cv::recoverPose(E, points1, points2, cameraMatrix, _R, _t, distanceThresh, cv::noArray(), triangulatedPoints);
}

double ftl::calibration::computeExtrinsicParameters(const Mat &K1, const Mat &D1,
	const Mat &K2, const Mat &D2, const vector<Point2d> &points1,
	const vector<Point2d> &points2, const vector<Point3d> &object_points, Mat &R,
	Mat &t, vector<Point3d> &points_out) {

	Mat F = cv::findFundamentalMat(points1, points2, cv::noArray(), cv::FM_8POINT);
	Mat E = K2.t() * F * K1;

	Mat points3dh;
	// distanceThresh unit?
	recoverPose(E, points1, points2, K1, K2, R, t, 1000.0, points3dh);

	points_out.clear();
	points_out.reserve(points3dh.cols);

	for (int col = 0; col < points3dh.cols; col++) {
		CHECK(points3dh.at<double>(3, col) != 0);
		Point3d p = Point3d(points3dh.at<double>(0, col),
							points3dh.at<double>(1, col),
							points3dh.at<double>(2, col))
							/ points3dh.at<double>(3, col);
		points_out.push_back(p);
	}

	double s = ftl::calibration::optimizeScale(object_points, points_out);
	t = t * s;

	auto params1 = Camera(K1, D1, Mat::eye(3, 3, CV_64FC1), Mat::zeros(3, 1, CV_64FC1));
	auto params2 = Camera(K2, D2, R, t);

	auto ba = BundleAdjustment();
	ba.addCamera(params1);
	ba.addCamera(params2);

	for (size_t i = 0; i < points_out.size(); i++) {
		ba.addPoint({points1[i], points2[i]}, points_out[i]);
	}

	double error_before = ba.reprojectionError();

	BundleAdjustment::Options options;
	options.optimize_intrinsic = false;
	options.fix_camera_extrinsic = {0};
	ba.run(options);

	double error_after = ba.reprojectionError();

	// bundle adjustment didn't work correctly if these checks fail
	if (error_before < error_after) {
		LOG(WARNING) << "error before < error_after (" << error_before << "  <" << error_after << ")";
	}
	CHECK((cv::countNonZero(params1.rvec()) == 0) && (cv::countNonZero(params1.tvec()) == 0));

	R = params2.rmat();
	t = params2.tvec();

	return sqrt(error_after);
}
