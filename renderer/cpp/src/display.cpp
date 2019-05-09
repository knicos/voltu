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

#if defined HAVE_PCL
	pclviz_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("FTL Cloud"));
	pclviz_->setBackgroundColor (255, 255, 255);
	pclviz_->addCoordinateSystem (1.0);
	pclviz_->setShowFPS(true);
	pclviz_->initCameraParameters ();

	pclviz_->registerKeyboardCallback (
		[](const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
			auto viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr*>(viewer_void);
			LOG(INFO) << "Key " << event.getKeySym ();
			pcl::visualization::Camera cam;
			viewer->getCameraParameters(cam);

			Eigen::Vector3f pos(cam.pos[0], cam.pos[1], cam.pos[2]);
			Eigen::Vector3f dir = Eigen::Vector3f(cam.focal[0], cam.focal[1], cam.focal[2]) - pos; //.normalize();
			dir.normalize();

			if (event.getKeySym() == "Up") {
				pos += 20.0f*dir;
			} else if (event.getKeySym() == "Down") {
				pos -= 20.0f*dir;
			} else if (event.getKeySym() == "Left") {
				// TODO Rotate -90
				pos += 20.0f*dir;
			} else if (event.getKeySym() == "Right") {
				// TODO Rotate 90
				pos += 20.0f*dir;
			}


			cam.pos[0] = pos[0];
			cam.pos[1] = pos[1];
			cam.pos[2] = pos[2];
			viewer->setCameraParameters(cam);

		}, (void*)&pclviz_);
#endif  // HAVE_PCL

	active_ = true;
}

Display::~Display() {
	#if defined HAVE_VIZ
	delete window_;
	#endif  // HAVE_VIZ
}

#if defined HAVE_PCL
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr matToPointXYZ(const cv::Mat &cvcloud, const cv::Mat &rgbimage) {
	/*
	*  Function: Get from a Mat to pcl pointcloud datatype
	*  In: cv::Mat
	*  Out: pcl::PointCloud
	*/

	//char pr=100, pg=100, pb=100;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);//(new pcl::pointcloud<pcl::pointXYZ>);

	for(int i=0;i<cvcloud.rows;i++) {
	for(int j=0;j<cvcloud.cols;j++) {
		//std::cout<<i<<endl;

		pcl::PointXYZRGB point;
		cv::Vec3f cvpoint = cvcloud.at<cv::Vec3f>(i,j);
		point.x = cvpoint[0];
		point.y = cvpoint[1];
		point.z = cvpoint[2];

		//LOG(INFO) << point.x;
		if (point.x == INFINITY || point.y == INFINITY || point.z == INFINITY || point.z < 600.0f) {
			point.x = 0; point.y = 0; point.z = 0;
		}

		cv::Point3_<uchar> prgb = rgbimage.at<cv::Point3_<uchar>>(i, j);

		// when color needs to be added:
		uint32_t rgb = (static_cast<uint32_t>(prgb.z) << 16 | static_cast<uint32_t>(prgb.y) << 8 | static_cast<uint32_t>(prgb.x));
		point.rgb = *reinterpret_cast<float*>(&rgb);

		point_cloud_ptr -> points.push_back(point);
	}
	}
	point_cloud_ptr->width = cvcloud.cols;
	point_cloud_ptr->height = cvcloud.rows;

	return point_cloud_ptr;
}
#endif  // HAVE_PCL

bool Display::render(const cv::Mat &rgb, const cv::Mat &depth) {
	Mat idepth;

	if (config_["points"] && q_.rows != 0) {
#if defined HAVE_PCL
		cv::Mat_<cv::Vec3f> XYZ(depth.rows, depth.cols);   // Output point cloud
		reprojectImageTo3D(depth, XYZ, q_, false);
		auto pc = matToPointXYZ(XYZ, rgb);

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
		if (!pclviz_->updatePointCloud<pcl::PointXYZRGB> (pc, rgb, "reconstruction")) {
			pclviz_->addPointCloud<pcl::PointXYZRGB> (pc, rgb, "reconstruction");
			pclviz_->setCameraPosition(-878.0, -71.0, -2315.0, -0.1, -0.99, 0.068, 0.0, -1.0, 0.0);
			pclviz_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reconstruction");
		}
#elif defined HAVE_VIZ
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

#if defined HAVE_PCL
bool Display::render(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc) {
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
	if (!pclviz_->updatePointCloud<pcl::PointXYZRGB> (pc, rgb, "reconstruction")) {
		pclviz_->addPointCloud<pcl::PointXYZRGB> (pc, rgb, "reconstruction");
	}

	pclviz_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
	return true;
}
#endif  // HAVE_PCL
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
		#if defined HAVE_PCL
		pclviz_->spinOnce(20);
		#elif defined HAVE_VIZ
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
	#if defined HAVE_PCL
	return active_ && !pclviz_->wasStopped();
	#elif defined HAVE_VIZ
	return active_ && !window_->wasStopped();
	#else
	return active_;
	#endif
}

