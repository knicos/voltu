#include "catch.hpp"

#include "calibration.hpp"
#include "optimization.hpp"

#include <vector>
#include <opencv2/core/core.hpp>

using std::vector;
using std::string;

using cv::Mat;
using cv::Point3d;
using cv::Point2d;

using namespace ftl::calibration;

void loadData(const string &fname,vector<Mat> &K, vector<vector<int>> &visible,
	vector<vector<Point2d>> &points) {

	cv::FileStorage fs(fname, cv::FileStorage::READ);

	fs["K"] >> K;
	fs["points2d"] >> points;
	fs["visible"] >> visible;
	fs.release();
}

/* TEST_CASE("Camera calibration and parameter optimization", "") {

    vector<Mat> K;
    vector<vector<int>> visible;
	vector<vector<Point2d>> observations;

	SECTION("Load data") {
		loadData("data/points.yml", K, visible, observations);
		//REQUIRE(K.size() != 0);
	}

	//int ncameras = K.size();
	//int npoints = observations[0].size();
}*/
