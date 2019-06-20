#include <ftl/virtual_source.hpp>
#include <ftl/depth_camera.hpp>
#include <ftl/scene_rep_hash_sdf.hpp>
#include <ftl/ray_cast_sdf.hpp>

#define LOGURU_WITH_STREAMS 1
#include <loguru.hpp>

using ftl::rgbd::VirtualSource;
using std::mutex;
using std::unique_lock;

VirtualSource::VirtualSource(ftl::rgbd::Source *host)
		: ftl::rgbd::detail::Source(host) {
	rays_ = ftl::create<CUDARayCastSDF>(host, "raycaster"); //new CUDARayCastSDF(host->getConfig());
	scene_ = nullptr;

	params_.fx = rays_->value("focal", 430.0f);
	params_.fy = params_.fx;
	params_.width = rays_->value("width", 640);
	params_.height = rays_->value("height", 480);
	params_.cx =  -((double)params_.width / 2);
	params_.cy = -((double)params_.height / 2);
	params_.maxDepth = rays_->value("max_depth", 10.0f);
	params_.minDepth = rays_->value("min_depth", 0.1f);

	rgb_ = cv::Mat(cv::Size(params_.width,params_.height), CV_8UC3);
	idepth_ = cv::Mat(cv::Size(params_.width,params_.height), CV_32SC1);
	depth_ = cv::Mat(cv::Size(params_.width,params_.height), CV_32FC1);
}

VirtualSource::~VirtualSource() {
	if (scene_) delete scene_;
	if (rays_) delete rays_;
}

void VirtualSource::setScene(ftl::voxhash::SceneRep *scene) {
	scene_ = scene;
}

bool VirtualSource::grab() {
	if (scene_) {
		DepthCameraParams params;
		params.fx = params_.fx;
		params.fy = params_.fy;
		params.mx = -params_.cx;
		params.my = -params_.cy;
		params.m_imageWidth = params_.width;
		params.m_imageHeight = params_.height;
		params.m_sensorDepthWorldMin = params_.minDepth;
		params.m_sensorDepthWorldMax = params_.maxDepth;

		// TODO(Nick) Use double precision pose here
		rays_->render(scene_->getHashData(), scene_->getHashParams(), params, host_->getPose().cast<float>());

		//unique_lock<mutex> lk(mutex_);
		if (rays_->isIntegerDepth()) {
			rays_->getRayCastData().download((int*)idepth_.data, (uchar3*)rgb_.data, rays_->getRayCastParams());
			idepth_.convertTo(depth_, CV_32FC1, 1.0f / 100.0f);
		} else {
			rays_->getRayCastData().download((int*)depth_.data, (uchar3*)rgb_.data, rays_->getRayCastParams());
		}
	}
	return true;
}

bool VirtualSource::isReady() {
	return true;
}
