#pragma once

#include <loguru.hpp>

#include <opencv2/core.hpp>
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
void drawEpipolarLines(vector<Point2d> const &points, Mat &img, Mat const &F, Scalar color, int image=1);


/* @breif	Find calibration points. AruCo markers, two per image.
 */
int findCorrespondingPoints(vector<Mat> imgs, vector<vector<Point2d>> &points,
							vector<int> &visible);

/* @brief	Find AruCo marker centers.
 * @param	(input) image
 * @param	(output) found centers
 * @param	(output) marker IDs
 */
void findMarkerCenters(Mat &img, vector<Point2d> &points, vector<int> &ids, int dict=cv::aruco::DICT_4X4_50);

/* OpenCV's recoverPose() expects both cameras to have identical intrinsic
 * parameters.
 * 
 * https://github.com/opencv/opencv/blob/371bba8f54560b374fbcd47e7e02f015ac4969ad/modules/calib3d/src/five-point.cpp#L461
 */
int recoverPose(Mat &E, vector<Point2d> &_points1, vector<Point2d> &_points2,
				Mat &_cameraMatrix1, Mat &_cameraMatrix2,
				Mat &_R, Mat &_t, double distanceThresh,
				Mat &triangulatedPoints);

/* @brief	Calculate RMS reprojection error
 * @param	3D points
 * @param	Expected 2D points
 * @param	Camera matrix
 * @param	Rotation matrix/vector
 * @param	Translation vector
 */
double reprojectionError(	const vector<Point3d> &points3d, const vector<Point2d> &points2d,
							const Mat &K, const Mat &rvec, const Mat &tvec);

inline double euclideanDistance(Point3d a, Point3d b) {
	Point3d c = a - b;
	return sqrt(c.x*c.x + c.y*c.y + c.z*c.z);
}

inline Point3d transformPoint(Point3d p, Mat R, Mat t) {
	DCHECK(R.size() == Size(3, 3));
	DCHECK(t.size() == Size(1, 3));
	return Point3d(Mat(R * Mat(p) + t));
}

inline Point3d inverseTransformPoint(Point3d p, Mat R, Mat t) {
	DCHECK(R.size() == Size(3, 3));
	DCHECK(t.size() == Size(1, 3));
	return Point3d(Mat(R.t() * (Mat(p) - t)));
}

inline Mat getMat4x4(const Mat &R, const Mat &t) {
	DCHECK(R.size() == Size(3, 3));
	DCHECK(t.size() == Size(1, 3));
	Mat M = Mat::eye(Size(4, 4), CV_64FC1);
	R.copyTo(M(cv::Rect(0, 0, 3, 3)));
	t.copyTo(M(cv::Rect(3, 0, 1, 3)));
	return M;
}

inline void getRT(const Mat RT, Mat &R, Mat &t) {
	R = RT(cv::Rect(0, 0, 3, 3));
	t = RT(cv::Rect(3, 0, 1, 3));
}

// calculate transforms from (R1, t1) to (R2, t2), where parameters
// (R1, t1) and (R2, t2) map to same (target) coordinate system

inline void calculateTransform(const Mat &R1, const Mat &T1, const Mat &R2, const Mat &T2, Mat &R, Mat &tvec, Mat &M) {
	Mat M_src = getMat4x4(R1, T1);
	Mat M_dst = getMat4x4(R2, T2);
	M = M_dst.inv() * M_src;	
	R = M(cv::Rect(0, 0, 3, 3));
	tvec = M(cv::Rect(3, 0, 1, 3));
}

inline void calculateTransform(const Mat &R1, const Mat &T1, const Mat &R2, const Mat &T2,Mat &R, Mat &tvec) {
	Mat M;
	calculateTransform(R1, T1, R2, T2, R, tvec, M);
}

inline void calculateInverse(const Mat &R2, const Mat &T2, Mat &R, Mat &T) {
	Mat R1 = Mat::eye(Size(3, 3), CV_64FC1);
	Mat T1(Size(1, 3), CV_64FC1, Scalar(0.0));
	calculateTransform(R1, T1, R2, T2, R, T);
}