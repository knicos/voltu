/*
 * Copyright 2019 Nicolas Pope
 */

#include <glog/logging.h>

#include <ftl/display.hpp>

using ftl::Display;
using cv::Mat;
using cv::Vec3f;

Display::Display(nlohmann::json &config) : config_(config) {
	#if defined HAVE_VIZ
	window_ = new cv::viz::Viz3d("FTL");
	window_->setBackgroundColor(cv::viz::Color::white());
	#endif  // HAVE_VIZ
	active_ = true;
}

Display::~Display() {
	#if defined HAVE_VIZ
	delete window_;
	#endif  // HAVE_VIZ
}

bool Display::render(const cv::Mat &rgb, const cv::Mat &depth) {
	Mat idepth;

	if (config_["points"] && q_.rows != 0) {
		#if defined HAVE_VIZ
		//cv::Mat Q_32F;
		//calibrate_.getQ().convertTo(Q_32F, CV_32F);
		cv::Mat_<cv::Vec3f> XYZ(depth.rows, depth.cols);   // Output point cloud
		reprojectImageTo3D(depth+20.0f, XYZ, q_, true);

		// Remove all invalid pixels from point cloud
		XYZ.setTo(Vec3f(NAN, NAN, NAN), depth == 0.0f);

		cv::viz::WCloud cloud_widget = cv::viz::WCloud(XYZ, rgb);
		cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 2);

		window_->showWidget("coosys", cv::viz::WCoordinateSystem());
		window_->showWidget("Depth", cloud_widget);

		//window_->spinOnce(40, true);

		#else  // HAVE_VIZ

		LOG(ERROR) << "Need OpenCV Viz module to display points";

		#endif  // HAVE_VIZ
	}

	if (config_["left"]) {
		cv::imshow("RGB", rgb);
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
		if ((bool)config_["flip_vert"]) {
			cv::flip(depth, idepth, 0);
		} else {
			idepth = depth;
		}

    	idepth.convertTo(idepth, CV_8U, 255.0f / 256.0f);  // TODO(nick)

    	applyColorMap(idepth, idepth, cv::COLORMAP_JET);
		cv::imshow("Disparity", idepth);
		//if(cv::waitKey(40) == 27) {
	        // exit if ESC is pressed
	    //    active_ = false;
	    //}
    }

	return true;
}

bool Display::render(const cv::Mat &img, style_t s) {
	if (s == STYLE_NORMAL) {
		cv::imshow("Image", img);
	} else if (s = STYLE_DISPARITY) {
		Mat idepth;

		if ((bool)config_["flip_vert"]) {
			cv::flip(img, idepth, 0);
		} else {
			idepth = img;
		}

    	idepth.convertTo(idepth, CV_8U, 255.0f / 256.0f);

    	applyColorMap(idepth, idepth, cv::COLORMAP_JET);
		cv::imshow("Disparity", idepth);
	}

	return true;
}

void Display::wait(int ms) {
	if (config_["points"] && q_.rows != 0) {
		#if defined HAVE_VIZ
		window_->spinOnce(1, true);
		#endif  // HAVE_VIZ
	}
	
	if (config_["disparity"]) {
		if(cv::waitKey(ms) == 27) {
	        // exit if ESC is pressed
	        active_ = false;
	    }
	}
}

bool Display::active() const {
	#if defined HAVE_VIZ
	return active_ && !window_->wasStopped();
	#else
	return active_;
	#endif
}

