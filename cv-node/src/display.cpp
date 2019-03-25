#include <ftl/display.hpp>

using ftl::Display;
using namespace cv;

Display::Display(const Calibrate &cal, nlohmann::json &config) : calibrate_(cal), config_(config) {
	window_ = new cv::viz::Viz3d("FTL");
	window_->setBackgroundColor(cv::viz::Color::white());
	active_ = true;
}

Display::~Display() {
	delete window_;
}

bool Display::render(const cv::Mat &rgb, const cv::Mat &depth) {
	Mat idepth;

	if (config_["points"]) {
		cv::Mat Q_32F; // = (Mat_<double>(4,4) << 1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1); //(4,4,CV_32F);
		calibrate_.getQ().convertTo(Q_32F,CV_32F);
		cv::Mat_<cv::Vec3f> XYZ(depth.rows,depth.cols);   // Output point cloud
		reprojectImageTo3D(depth, XYZ, Q_32F, false);
		
		//cv::imshow("Points",XYZ);
		
		cv::viz::WCloud cloud_widget = cv::viz::WCloud( XYZ, rgb );
		cloud_widget.setRenderingProperty( cv::viz::POINT_SIZE, 2 );
		
		/* Rotation using rodrigues */
	    /// Rotate around (1,1,1)
	    /*rot_vec.at<float>(0,0) = 0.0f;
	    rot_vec.at<float>(0,1) = 0.0f;
	    rot_vec.at<float>(0,2) = CV_PI * 1.0f;*/

	    //Mat rot_mat;
	    //Rodrigues(rot_vec, rot_mat);

	    /// Construct pose
	    //Affine3f pose(rot_mat, Vec3f(0, 20.0, 0));
		window_->showWidget( "coosys", viz::WCoordinateSystem() );
		window_->showWidget( "Depth", cloud_widget );
		//window_->setWidgetPose("Depth", pose);

		window_->spinOnce( 1, true );
	}
	
	if (config_["depth"]) {
		/*Mat depth32F = (focal * (float)l.cols * base_line) / depth;
		normalize(depth32F, depth32F, 0, 255, NORM_MINMAX, CV_8U);
		cv::imshow("Depth", depth32F);
		if(cv::waitKey(10) == 27){
	        //exit if ESC is pressed
	       	active_ = false;
	    }*/
    } else if (config_["disparity"]) {
    	//if (disparity32F.type() == CV_32FC1) {
    		//disparity32F = disparity32F / (float)config["disparity"]["maximum"];
    		depth.convertTo(idepth,CV_8U,255.0f / 208.0f); //(float)config_["maximum"]);
    	//}
    	//normalize(disparity32F, disparity32F, 0, 255, NORM_MINMAX, CV_8U);
    	applyColorMap(idepth, idepth, cv::COLORMAP_JET);
		cv::imshow("Disparity", idepth);
		if(cv::waitKey(10) == 27){
	        //exit if ESC is pressed
	        active_ = false;
	    }
    }

	return true;
}

bool Display::active() const {
	return active_ && !window_->wasStopped();
}

