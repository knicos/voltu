//Uncomment the following line if you are compiling this code in Visual Studio
//#include "stdafx.h"

#include <gflags/gflags.h>

DEFINE_int32(grid_x, 1, "Horizontal video grid size");
DEFINE_int32(grid_y, 1, "Vertical video grid size");
DEFINE_int32(count, 1, "Number of video feeds");

#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace cv::sfm;
using namespace cv::xfeatures2d;
using namespace std;

void get_tracks(const vector<DMatch> &matches, const vector<KeyPoint> &kp1,
		const vector<KeyPoint> &kp2, vector<vector<Vec2d>> &tracks) {
		
	for (size_t i=0; i<matches.size(); i++) {
		// TODO Generalise to any number of keypoint frames
		Point2f point1 = kp1[matches[i].queryIdx].pt;
    	Point2f point2 = kp2[matches[i].trainIdx].pt;
    	
    	vector<Vec2d> track;
    	track.push_back(Vec2d(point1.x,point1.y));
    	track.push_back(Vec2d(point2.x,point2.y));
    	tracks.push_back(track);
    }
}

void convert_tracks(const vector<vector<Vec2d>> &tracks, vector<Mat> &points2d) {
	int n_frames = 2;
	int n_tracks = tracks.size();
	
	for (int i = 0; i < n_frames; ++i)
	{
		Mat_<double> frame(2, n_tracks);
		for (int j = 0; j < n_tracks; ++j)
		{
			frame(0,j) = tracks[j][i][0];
			frame(1,j) = tracks[j][i][1];
		}
		points2d.push_back(Mat(frame));
	}
}

int main(int argc, char* argv[])
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	
	auto gridX = FLAGS_grid_x;
	auto gridY = FLAGS_grid_y;
	auto count = FLAGS_count;

	bool cam = argc <= 1;
	Mat frame, gray, img;
	
    //vector<vector<Point> > cnts;
    VideoCapture *camA = NULL;
    VideoCapture *camB = NULL;
    Mat left,right;
    
    int minHessian = 400;
	Ptr<SURF> detector = SURF::create();
	detector->setHessianThreshold(minHessian);
    
    if (!cam) camA = new VideoCapture(argv[1]);
    else {
    	//camA = new VideoCapture(0); //open camera
    	//camB = new VideoCapture(1);
    }

	viz::Viz3d window("Coordinate Frame");
             window.setWindowSize(Size(500,500));
             window.setWindowPosition(Point(150,150));
             window.setBackgroundColor(); // black by default

    while(camA->read(frame)) {
    	vector<Mat> frames;
    	int resX = frame.cols / gridX;
    	int resY = frame.rows / gridY;
    	
    	// Split the video grid into individual feeds
    	if (!cam) {
    		int tcount = 0;
    		for (int x=0; x<gridX; x++) {
				for (int y=0; y<gridY; y++) {
					tcount++;
					Mat f(frame, Rect(x*resX,y*resY,(x+1)*resX,(y+1)*resY));
					cvtColor(f, f, COLOR_BGR2GRAY);
					frames.push_back(f);
					if (tcount == count) break;
				}
    			if (tcount == count) break;
    		}
    	} else {
    		//left = frame;
    		//camB->read(right);
    	}

    	// --- Features ---
	    
		//-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
		vector<vector<KeyPoint>> keypoints;
		vector<Mat> descriptors;
		
		for (auto x : frames) {
			vector<KeyPoint> kps;
			Mat desc;
			detector->detectAndCompute(x, Mat(), kps, desc);
			keypoints.push_back(kps);
			descriptors.push_back(desc);
		}
		
		// TODO This can be done on node machines...
		
		//-- Step 2: Matching descriptor vectors using FLANN matcher
		// Match between each sequential pair of images in the feed.
		FlannBasedMatcher matcher;
		std::vector< DMatch > matches;
		matcher.match( descriptors_1, descriptors_2, matches );
		double max_dist = 0; double min_dist = 100;
		//-- Quick calculation of max and min distances between keypoints
		for( int i = 0; i < descriptors_1.rows; i++ )
		{
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}
		
		std::vector< DMatch > good_matches;
		for( int i = 0; i < descriptors_1.rows; i++ )
		{ if( matches[i].distance <= max(2*min_dist, 0.02) )
		{ good_matches.push_back( matches[i]); }
		}
    	
    	// --- SFM ---
    	
    	std::vector<Mat> points2d;
    	vector<vector<Vec2d>> tracks;
    	get_tracks(good_matches, keypoints_1, keypoints_2, tracks);
    	convert_tracks(tracks, points2d);
    	
    	cout << "Tracks: " << tracks.size() << endl;
    	
    	 // Build instrinsics
		float f  = 5,
				cx = 200, cy = 200;
		Matx33d K = Matx33d( f, 0, cx,
				               0, f, cy,
				               0, 0,  1);
		
		bool is_projective = true;
		vector<Mat> Rs_est, ts_est, points3d_estimated;
		cv::sfm::reconstruct(points2d, Rs_est, ts_est, K, points3d_estimated, is_projective);
        
        vector<Vec3f> point_cloud_est;
		for (unsigned int i = 0; i < points3d_estimated.size(); ++i)
			point_cloud_est.push_back(Vec3f(points3d_estimated[i]));
        
        if ( point_cloud_est.size() > 0 )
		{
		viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
		window.showWidget("point_cloud", cloud_widget);
		} else {
		cout << "No points" << std::endl;
		}
        
        if(waitKey(1) == 27){
            //exit if ESC is pressed
            break;
        }

		//firstFrame = gray.clone();
		//GaussianBlur(gray, firstFrame, Size(21, 21), 0);
    	break;
    }
    
    window.spin();

	if (camA) delete camA;
	if (camB) delete camB;
    return 0;

}

