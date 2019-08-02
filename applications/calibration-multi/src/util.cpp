#include "util.hpp"

#include <loguru.hpp>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

using std::vector;

using cv::Mat;
using cv::Point2i;
using cv::Point2d;
using cv::Point3d;
using cv::Size;
using cv::Scalar;

/* @brief	Visualize epipolar lines for given points in the other image.
 * @param	Points in image
 * @param	Corresponding image where to draw the lines
 * @param	Fundamental matrix
 * @param	Line color
 * @param	Which image (1 or 2), see OpenCV's computeCorrespondEpilines()
 */
void drawEpipolarLines(vector<Point2d> const &points, Mat &img, Mat const &F, Scalar color, int image) {
	Mat lines;
	cv::computeCorrespondEpilines(points, image, F, lines);

	for (int i = 0; i < lines.rows; i++) {
		cv::Vec3f l = lines.at<cv::Vec3f>(i);
		float a = l[0];
		float b = l[1];
		float c = l[2];
		float x0, y0, x1, y1;
		x0 = 0;
		y0 = (-c -a * x0) / b;
		x1 = img.cols;
		y1 = (-c -a * x1) / b;
		cv::line(img, cv::Point(x0, y0), cv::Point(x1,y1), color, 1);
	}
}

/* @breif	Find calibration points. AruCo markers, two per image.
 *			visible parameter input/ouput
 */
int findCorrespondingPoints(vector<Mat> imgs, vector<vector<Point2d>> &points,
							vector<int> &visible) {
	using namespace cv;
	int count = 0;

	visible.resize(imgs.size(), 1);

	points.clear();
	points.resize(imgs.size(), vector<Point2d>(2, Point2d(0.0, 0.0)));

	auto dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);
	vector<vector<Point2f>> corners;
	vector<int> ids;
	
	for (size_t i = 0; i < imgs.size(); i++) {
		if (visible[i] == 0) continue;

		aruco::detectMarkers(imgs[i], dictionary, corners, ids);
		if (corners.size() == 2) {
			Point2d center0((corners[0][0] + corners[0][1] + corners[0][2] + corners[0][3]) / 4.0);
			Point2d center1((corners[1][0] + corners[1][1] + corners[1][2] + corners[1][3]) / 4.0);
			if (ids[0] != 0) { std::swap(center0, center1); }

			points[i][0] = center0; points[i][1] = center1;
			visible[i] = 1;

			count++;
		}
		else {
			visible[i] = 0;
		}
	}

	return count;
}

/* @brief	Find AruCo marker centers.
 * @param	(input) image
 * @param	(output) found centers
 * @param	(output) marker IDs
 */
void findMarkerCenters(Mat &img, vector<Point2d> &points, vector<int> &ids, int dict) {
	using namespace cv;

	points.clear();

	auto dictionary = aruco::getPredefinedDictionary(dict);
	vector<vector<Point2f>> corners;

	aruco::detectMarkers(img, dictionary, corners, ids);
	for (size_t j = 0; j < corners.size(); j++) {
		Point2f center((corners[j][0] + corners[j][1] + corners[j][2] + corners[j][3]) / 4.0);
		points.push_back(center);
	}
}

/* OpenCV's recoverPose() expects both cameras to have identical intrinsic
 * parameters.
 */
int recoverPose(Mat &E, vector<Point2d> &_points1, vector<Point2d> &_points2,
				Mat &_cameraMatrix1, Mat &_cameraMatrix2,
				Mat &_R, Mat &_t, double distanceThresh,
				Mat &triangulatedPoints) {

	Mat points1, points2, cameraMatrix1, cameraMatrix2, cameraMatrix;
	
	Mat(_points1.size(), 2, CV_64FC1, _points1.data()).convertTo(points1, CV_64F);
	Mat(_points2.size(), 2, CV_64FC1, _points2.data()).convertTo(points2, CV_64F);
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

/* @brief	Calculate RMS reprojection error
 * @param	3D points
 * @param	Expected 2D points
 * @param	Camera matrix
 * @param	Rotation matrix/vector
 * @param	Translation vector
 */
double reprojectionError(	const vector<Point3d> &points3d, const vector<Point2d> &points2d,
							const Mat &K, const Mat &rvec, const Mat &tvec) {
	
	DCHECK(points3d.size() == points2d.size());
	
	Mat _rvec;
	if (rvec.size() == Size(3, 3)) { cv::Rodrigues(rvec, _rvec); }
	else { _rvec = rvec; }

	DCHECK(_rvec.size() == Size(1, 3) || _rvec.size() == Size(3, 1));

	vector<Point2d> points_reprojected;
	cv::projectPoints(points3d, _rvec, tvec, K, cv::noArray(), points_reprojected);
	
	int n_points = points2d.size();
	double err = 0.0;

	for (int i = 0; i < n_points; i++) {
		Point2d a = points2d[i] - points_reprojected[i];
		err += a.x * a.x + a.y * a.y;
	}

	return sqrt(err / n_points);
}