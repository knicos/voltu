#include <ftl/display.hpp>
#include <glog/logging.h>

using ftl::Display;
using ftl::Calibrate;
using namespace cv;

Display::Display(const Calibrate &cal, nlohmann::json &config) : calibrate_(cal), config_(config) {
	#if defined HAVE_VIZ
	window_ = new cv::viz::Viz3d("FTL");
	window_->setBackgroundColor(cv::viz::Color::white());
	#endif // HAVE_VIZ
	active_ = true;
}

Display::~Display() {
	#if defined HAVE_VIZ
	delete window_;
	#endif // HAVE_VIZ
}

bool Display::render(const cv::Mat &rgb, const cv::Mat &depth) {
	Mat idepth;

	if (config_["points"]) {
		#if defined HAVE_VIZ
		cv::Mat Q_32F;
		calibrate_.getQ().convertTo(Q_32F,CV_32F);
		cv::Mat_<cv::Vec3f> XYZ(depth.rows,depth.cols);   // Output point cloud
		reprojectImageTo3D(depth+20.0f, XYZ, Q_32F, true);
		
		// Remove all invalid pixels from point cloud
		XYZ.setTo(Vec3f(NAN,NAN,NAN), depth == 0.0f);
		
		cv::viz::WCloud cloud_widget = cv::viz::WCloud( XYZ, rgb );
		cloud_widget.setRenderingProperty( cv::viz::POINT_SIZE, 2 );
		
		window_->showWidget( "coosys", viz::WCoordinateSystem() );
		window_->showWidget( "Depth", cloud_widget );

		window_->spinOnce( 1, true );

		#else // HAVE_VIZ

		LOG(ERROR) << "Need OpenCV Viz module to display points";

		#endif // HAVE_VIZ
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
	#if defined HAVE_VIZ
	return active_ && !window_->wasStopped();
	#else
	return active_;
	#endif
}

