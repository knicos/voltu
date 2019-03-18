#include <opencv2/opencv.hpp>
#include <ftl/local.hpp>
#include <ftl/synched.hpp>
#include <ftl/calibrate.hpp>
#include <ftl/disparity.hpp>
#include <nlohmann/json.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <glog/logging.h>

#include <string>
#include <map>
#include <vector>

using namespace ftl;
using std::string;
using std::vector;
using std::map;
using cv::Mat;
using json = nlohmann::json;
using std::ifstream;
using namespace cv;

// Store loaded configuration
static json config;

/**
 * Find and load a JSON configuration file
 */
static bool findConfiguration(const string &file) {
	// TODO Check other locations
	ifstream i((file != "") ? file : FTL_CONFIG_ROOT "/config.json");
	if (!i.is_open()) return false;
	i >> config;
	return true;
}

/**
 * Generate a map from command line option to value
 */
map<string,string> read_options(char ***argv, int *argc) {
	map<string,string> opts;
	
	while (*argc > 0) {
		string cmd((*argv)[0]);
		if (cmd[0] != '-') break;
		
		size_t p;
		if ((p = cmd.find("=")) == string::npos) {
			opts[cmd.substr(2)] = "true";
		} else {
			opts[cmd.substr(2,p-2)] = cmd.substr(p+1);
		}
		
		(*argc)--;
		(*argv)++;
	}
	
	return opts;
}

/**
 * Put command line options into json config. If config element does not exist
 * or is of a different type then report an error.
 */
static void process_options(const map<string,string> &opts) {
	for (auto opt : opts) {
		if (opt.first == "config") continue;
		
		try {
			auto ptr = json::json_pointer("/"+opt.first);
			auto v = json::parse(opt.second);
			if (v.type() != config.at(ptr).type()) {
				LOG(ERROR) << "Incorrect type for argument " << opt.first;
				continue;
			}
			config.at(ptr) = v;
		} catch(...) {
			LOG(ERROR) << "Unrecognised option: " << opt.first;
		}
	}
}

int main(int argc, char **argv) {
	argc--;
	argv++;
	
	// Process Arguments
	auto options = read_options(&argv, &argc);
	if (!findConfiguration(options["config"])) {
		LOG(FATAL) << "Could not find any configuration!";
	}
	process_options(options);
	
	// TODO Initiate the network
	
	LocalSource *lsrc;
	if (argc) {
		// Load video file
		lsrc = new LocalSource(argv[0], config["source"]);
	} else {
		// Use cameras
		lsrc = new LocalSource(config["source"]);
	}
	
	auto sync = new SyncSource(); // TODO Pass protocol object
	// Add any remote channels
	/*for (auto c : OPTION_channels) {
		sync->addChannel(c);
	}*/
	
	// Perform or load calibration intrinsics + extrinsics
	Calibrate calibrate(lsrc, config["calibration"]);
	if (config["calibrate"]) calibrate.recalibrate();
	if (!calibrate.isCalibrated()) LOG(WARNING) << "Cameras are not calibrated!";
    
    // Choose and configure disparity algorithm
    auto disparity = Disparity::create(config["disparity"]);
	
	Mat l, r, disparity32F, depth32F, lbw, rbw;
	
	cv::viz::Viz3d myWindow("FTL");
	myWindow.setBackgroundColor(cv::viz::Color::white());
	
	float base_line = (float)config["camera"]["base_line"];
	float focal = (float)(config["camera"]["focal_length"]) / (float)(config["camera"]["sensor_width"]);
	Mat rot_vec = Mat::zeros(1,3,CV_32F);
	
	while (!myWindow.wasStopped()) {
		// Read calibrated images.
		calibrate.undistort(l,r);
		
		// Feed into sync buffer and network forward
		sync->feed(LEFT, l,lsrc->getTimestamp());
		sync->feed(RIGHT, r,lsrc->getTimestamp());
		
		// Read back from buffer
		sync->get(LEFT,l);
		sync->get(RIGHT,r);
		
		// Black and white
        cvtColor(l,  lbw,  COLOR_BGR2GRAY);
        cvtColor(r, rbw, COLOR_BGR2GRAY);
        
        disparity->compute(lbw,rbw,disparity32F);
		//LOG(INFO) << "Disparity complete ";
		
		disparity32F.convertTo(disparity32F, CV_32F);
		disparity32F += 10.0f;
		
		// Clip the left edge
		Rect rect((int)config["disparity"]["maximum"],7,disparity32F.cols-(int)config["disparity"]["maximum"],disparity32F.rows-14);
		disparity32F = disparity32F(rect);
		l = l(rect);
		
		// HACK to make bad pixels invisible.
		normalize(disparity32F, depth32F, 0, 255, NORM_MINMAX, CV_8U);
		r = Mat(l.size(), CV_8UC3, Vec3i(255,255,255));
		l.copyTo(r,depth32F);
		
		// TODO Send RGB+D data somewhere
		
		// Convert disparity to depth

		//normalize(disparity32F, disparity32F, 0, 255, NORM_MINMAX, CV_8U);
		
		//cv::imshow("Disparity",filtered_disp);
		//Mat i3d;
		//const Mat &Q = calibrate.getQ();
		

		if (config["display"]["points"]) {
			cv::Mat Q_32F; // = (Mat_<double>(4,4) << 1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1); //(4,4,CV_32F);
			calibrate.getQ().convertTo(Q_32F,CV_32F);
			cv::Mat_<cv::Vec3f> XYZ(disparity32F.rows,disparity32F.cols);   // Output point cloud
			reprojectImageTo3D(disparity32F, XYZ, Q_32F, false);
			
			//cv::imshow("Points",XYZ);
			
			cv::viz::WCloud cloud_widget = cv::viz::WCloud( XYZ, r );
			cloud_widget.setRenderingProperty( cv::viz::POINT_SIZE, 2 );
			
			/* Rotation using rodrigues */
		    /// Rotate around (1,1,1)
		    rot_vec.at<float>(0,0) = 0.0f;
		    rot_vec.at<float>(0,1) = 0.0f;
		    rot_vec.at<float>(0,2) = CV_PI * 1.0f;

		    Mat rot_mat;
		    Rodrigues(rot_vec, rot_mat);

		    /// Construct pose
		    Affine3f pose(rot_mat, Vec3f(0, 20.0, 0));
			myWindow.showWidget( "coosys", viz::WCoordinateSystem() );
			myWindow.showWidget( "Depth", cloud_widget );
			myWindow.setWidgetPose("Depth", pose);

			myWindow.spinOnce( 30, true );
		}
		
		if (config["display"]["depth"]) {
			depth32F = (focal * (float)l.cols * base_line) / disparity32F;
			normalize(depth32F, depth32F, 0, 255, NORM_MINMAX, CV_8U);
			cv::imshow("Depth", depth32F);
			if(cv::waitKey(10) == 27){
		        //exit if ESC is pressed
		        break;
		    }
        } else if (config["display"]["disparity"]) {
        	normalize(disparity32F, disparity32F, 0, 255, NORM_MINMAX, CV_8U);
			cv::imshow("Disparity", disparity32F);
			if(cv::waitKey(10) == 27){
		        //exit if ESC is pressed
		        break;
		    }
        }
	}
}

