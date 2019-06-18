#include "sfm.hpp"
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

using std::vector;
using std::tuple;

using namespace cv;
using namespace cv::sfm;
using namespace cv::xfeatures2d;
using ftl::registration::CalibrationChessboard;

CalibrationChessboard::CalibrationChessboard(ftl::Configurable *root) {
	pattern_size_ = Size(9, 6);
	image_size_ = Size(1280, 720);
	pattern_square_size_ = 36.0;//0.036;
	// CALIB_CB_NORMALIZE_IMAfE | CALIB_CB_EXHAUSTIVE | CALIB_CB_ACCURACY 
	chessboard_flags_ = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY | cv::CALIB_CB_EXHAUSTIVE;
}

void CalibrationChessboard::objectPoints(vector<Vec3f> &out) {
	out.reserve(pattern_size_.width * pattern_size_.height);
	for (int row = 0; row < pattern_size_.height; ++row) {
	for (int col = 0; col < pattern_size_.width; ++col) {
		out.push_back(Vec3f(col * pattern_square_size_, row * pattern_square_size_, 0));
	}}
}

bool CalibrationChessboard::findPoints(Mat &img, vector<Vec2f> &points) {
	return cv::findChessboardCornersSB(img, pattern_size_, points, chessboard_flags_);
}

void CalibrationChessboard::drawPoints(Mat &img, const vector<Vec2f> &points) {
	cv::drawChessboardCorners(img, pattern_size_, points, true);
}

static void get_tracks(const vector<vector<DMatch>> &matches, const vector<KeyPoint> &kp1,
		const vector<KeyPoint> &kp2, vector<vector<Vec2d>> &tracks, int k) {
		
	for (size_t i=0; i<matches.size(); i++) {
		// TODO Generalise to any number of keypoint frames
		if (k >= matches[i].size()) continue;
		Point2f point1 = kp1[matches[i][k].queryIdx].pt;
    	Point2f point2 = kp2[matches[i][k].trainIdx].pt;
    	
    	vector<Vec2d> track;
    	track.push_back(Vec2d(point1.x,point1.y));
    	track.push_back(Vec2d(point2.x,point2.y));
    	tracks.push_back(track);
    }
}

static void get_tracks(const vector<DMatch> &matches, const vector<KeyPoint> &kp1,
		const vector<KeyPoint> &kp2, vector<vector<Vec2d>> &tracks) {
		
	for (size_t i=0; i<matches.size(); i++) {
		// TODO Generalise to any number of keypoint frames
		//if (k >= matches[i].size()) continue;
		Point2f point1 = kp1[matches[i].queryIdx].pt;
    	Point2f point2 = kp2[matches[i].trainIdx].pt;
    	
    	vector<Vec2d> track;
    	track.push_back(Vec2d(point1.x,point1.y));
    	track.push_back(Vec2d(point2.x,point2.y));
    	tracks.push_back(track);
    }
}

static void convert_tracks(const vector<vector<Vec2d>> &tracks, vector<tuple<int,int,int,int>> &points2d) {
	//int n_frames = 2;
	int n_tracks = tracks.size();
	
	//for (int i = 0; i < n_frames; ++i)
	//{
		//Mat_<double> frame(2, n_tracks);
		for (int j = 0; j < n_tracks; ++j)
		{
			//frame(0,j) = tracks[j][i][0];
			//frame(1,j) = tracks[j][i][1];
			points2d.push_back(std::make_tuple(tracks[j][0][0], tracks[j][0][1], tracks[j][1][0], tracks[j][1][1]));
		}
	//}
}

bool ftl::registration::featuresChess(cv::Mat &frame1, cv::Mat &frame2, std::vector<std::tuple<int,int,int,int>> &points) {
	CalibrationChessboard cb(nullptr);

	vector<Vec2f> points1, points2;
	if (!cb.findPoints(frame1, points1)) {
		LOG(ERROR) << "Could not find chessboard (1)";
		return false;
	}
	if (!cb.findPoints(frame2, points2)) {
		LOG(ERROR) << "Could not find chessboard (2)";
		return false;
	}

	if (points1.size() != points2.size()) return false;

	for (size_t i=0; i<points1.size(); i++) {
		points.push_back(std::make_tuple(points1[i][0], points1[i][1], points2[i][0], points2[i][1]));
	}
	return true;
}

bool ftl::registration::featuresSIFT(cv::Mat &frame1, cv::Mat &frame2, std::vector<std::tuple<int,int,int,int>> &points, int K) {
	int minHessian = 400;
	Ptr<SIFT> detector = SIFT::create();
	//detector->setHessianThreshold(minHessian);

	vector<vector<KeyPoint>> keypoints;
	vector<Mat> descriptors;
	
	{
		vector<KeyPoint> kps;
		Mat desc;
		detector->detectAndCompute(frame1, Mat(), kps, desc);
		keypoints.push_back(kps);
		descriptors.push_back(desc);
	}

	{
		vector<KeyPoint> kps;
		Mat desc;
		detector->detectAndCompute(frame2, Mat(), kps, desc);
		keypoints.push_back(kps);
		descriptors.push_back(desc);
	}
	
	// TODO This can be done on node machines...
	
	//-- Step 2: Matching descriptor vectors using FLANN matcher
	// Match between each sequential pair of images in the feed.
	FlannBasedMatcher matcher;
	std::vector<std::vector< DMatch >> matches;
	matcher.knnMatch( descriptors[0], descriptors[1], matches, K );
	
	const float ratio_thresh = 0.7f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++)
    {
		for (int k=0; k<K-1; k++) {
			if (matches[i][k].distance < ratio_thresh * matches[i][k+1].distance)
			{
				good_matches.push_back(matches[i][k]);
			} else break;
		}
    }
	
	// --- SFM ---

	//for (int i=0; i<K; i++) {
		vector<vector<Vec2d>> tracks;
		get_tracks(good_matches, keypoints[0], keypoints[1], tracks);
		convert_tracks(tracks, points);
	//}

	//cout << "Tracks: " << tracks.size() << endl;
	return true;
}
