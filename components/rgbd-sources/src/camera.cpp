#include <ftl/rgbd/camera.hpp>

using ftl::rgbd::Camera;

Camera Camera::from(ftl::Configurable *cfg) {
	Camera r;
	r.width = cfg->value("width", 1280);
	r.height = cfg->value("height", 720);
	r.fx = cfg->value("focal", 700.0f);
	r.fy = r.fx;
	r.cx = -(float)r.width / 2.0f;
	r.cy = -(float)r.height / 2.0f;
	r.minDepth = cfg->value("minDepth", 0.1f);
	r.maxDepth = cfg->value("maxDepth", 15.0f);
	r.doffs = 0;
	r.baseline = cfg->value("baseline", 0.05f);
	return r;
}

cv::Mat Camera::getCameraMatrix() const {
	cv::Mat K = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
	K.at<double>(0,0) = fx;
	K.at<double>(0,2) = -cx;
	K.at<double>(1,1) = fy;
	K.at<double>(1,2) = -cy;
	return K;
}

/*
 * Scale camera parameters to match resolution.
 */
Camera Camera::scaled(int width, int height) const {
	const auto &cam = *this;
	float scaleX = (float)width / (float)cam.width;
	float scaleY = (float)height / (float)cam.height;

	//CHECK( abs(scaleX - scaleY) < 0.00000001f );

	Camera newcam = cam;
	newcam.width = width;
	newcam.height = height;
	newcam.fx *= scaleX;
	newcam.fy *= scaleY;
	newcam.cx *= scaleX;
	newcam.cy *= scaleY;
	newcam.doffs *= scaleX;

	return newcam;
}
