#include <ftl/rgbd/camera.hpp>
#include <ftl/rgbd/capabilities.hpp>

using ftl::rgbd::Camera;
using ftl::rgbd::Capability;

// TODO: Put in better place?
std::string ftl::rgbd::capabilityName(Capability c) {
	switch (c) {
	case Capability::MOVABLE		: return "movable";
	case Capability::ACTIVE			: return "active";
	case Capability::VIDEO			: return "video";
	case Capability::ADJUSTABLE		: return "adjustable";
	case Capability::VIRTUAL		: return "virtual";
	case Capability::TOUCH			: return "touch";
	case Capability::VR				: return "vr";
	case Capability::LIVE			: return "live";
	case Capability::FUSED			: return "fused";
	default: return "Unknown";
	}
}

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

cv::Mat Camera::getCameraMatrix(const cv::Size& sz) const {
	if (sz == cv::Size{0, 0}) {
		cv::Mat K = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
		K.at<double>(0,0) = fx;
		K.at<double>(0,2) = -cx;
		K.at<double>(1,1) = fy;
		K.at<double>(1,2) = -cy;
		return K;
	}
	else {
		return scaled(sz.width, sz.height).getCameraMatrix();
	}
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

/*Eigen::Vector4f ftl::rgbd::Camera::eigenScreenToCam(int ux, int uy, float depth) const {
	const float x = static_cast<float>(((float)ux+cx) / fx);
	const float y = static_cast<float>(((float)uy+cy) / fy);
	Eigen::Vector4f v;
	v[0] = depth*x;
	v[1] = depth*y;
	v[2] = depth;
	v[3] = 1.0f;
	return v;
}*/
