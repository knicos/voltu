//Uncomment the following line if you are compiling this code in Visual Studio
//#include "stdafx.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

int main(int argc, char* argv[])
{
	bool cam = argc <= 1;
	Mat frame, gray, img;
	
    //vector<vector<Point> > cnts;
    VideoCapture *camA = NULL;
    VideoCapture *camB = NULL;
    
    if (!cam) camA = new VideoCapture(argv[1]);
    else {
    	camA = new VideoCapture(0); //open camera
    	camB = new VideoCapture(1);
    }

	auto sift = xfeatures2d::SiftFeatureDetector::create();
	vector<KeyPoint> kp;
	
	Mat left, right;
	int minHessian = 400;
	Ptr<SURF> detector = SURF::create();
	detector->setHessianThreshold(minHessian);

    while(camA->read(frame)) {
    	if (!cam) {
    		// Stereo
    		left = Mat(frame, Rect(0,0,frame.cols/2,frame.rows));
    		right = Mat(frame, Rect(0,0,frame.cols/2,frame.rows));
    	} else {
    		left = frame;
    		camB->read(right);
    	}
    	
    	//convert to grayscale
	    cvtColor(left, left, COLOR_BGR2GRAY);
	    cvtColor(right, right, COLOR_BGR2GRAY);
	    
		//-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
		std::vector<KeyPoint> keypoints_1, keypoints_2;
		Mat descriptors_1, descriptors_2;
		detector->detectAndCompute( left, Mat(), keypoints_1, descriptors_1 );
		detector->detectAndCompute( right, Mat(), keypoints_2, descriptors_2 );
		//-- Step 2: Matching descriptor vectors using FLANN matcher
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
		//-- Draw only "good" matches
		Mat img_matches;
		drawMatches( left, keypoints_1, right, keypoints_2,
				   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		//-- Show detected matches
		imshow( "Good Matches", img_matches );
        
        if(waitKey(1) == 27){
            //exit if ESC is pressed
            break;
        }

		//firstFrame = gray.clone();
		//GaussianBlur(gray, firstFrame, Size(21, 21), 0);
    
    }

	if (camA) delete camA;
	if (camB) delete camB;
    return 0;

}

