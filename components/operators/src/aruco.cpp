#include "ftl/operators/detectandtrack.hpp"
#include "ftl/codecs/shapes.hpp"

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <ftl/profiler.hpp>
#include <loguru.hpp>

using ftl::operators::ArUco;
using ftl::rgbd::Frame;

using ftl::codecs::Channel;
using ftl::codecs::Shape3D;

using cv::Mat;
using cv::Point2f;
using cv::Vec3d;

using std::vector;
using std::list;

static cv::Mat rmat(cv::Vec3d &rvec) {
	cv::Mat R(cv::Size(3, 3), CV_64FC1);
	cv::Rodrigues(rvec, R);
	return R;
}

static Eigen::Matrix4d matrix(cv::Vec3d &rvec, cv::Vec3d &tvec) {
	cv::Mat M = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
	rmat(rvec).copyTo(M(cv::Rect(0, 0, 3, 3)));
	M.at<double>(0, 3) = tvec[0];
	M.at<double>(1, 3) = tvec[1];
	M.at<double>(2, 3) = tvec[2];
	Eigen::Matrix4d r;
	cv::cv2eigen(M,r);
	return r;
}

ArUco::ArUco(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {
	dictionary_ = cv::aruco::getPredefinedDictionary(cfg->value("dictionary", 0));
	params_ = cv::aruco::DetectorParameters::create();
	params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
	params_->cornerRefinementMinAccuracy = 0.01;
	params_->cornerRefinementMaxIterations = 20;

	channel_in_ = Channel::Colour;
	channel_out_ = Channel::Shapes3D;

	cfg->on("dictionary", [this,cfg]() {
		dictionary_ = cv::aruco::getPredefinedDictionary(cfg->value("dictionary", 0));
	});
}

bool ArUco::apply(Frame &in, Frame &out, cudaStream_t) {
	if (!in.hasChannel(channel_in_)) { return false; }

	estimate_pose_ = config()->value("estimate_pose", true);
	marker_size_ = config()->value("marker_size", 0.1f);

	std::vector<Vec3d> rvecs;
	std::vector<Vec3d> tvecs;
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> ids;

	{
		FTL_Profile("ArUco", 0.02);
		cv::cvtColor(in.get<cv::Mat>(channel_in_), tmp_, cv::COLOR_BGRA2GRAY);

		const Mat K = in.getLeftCamera().getCameraMatrix();
		const Mat dist;

		cv::aruco::detectMarkers(tmp_, dictionary_,
								corners, ids, params_, cv::noArray(), K, dist);

		if (estimate_pose_) {
			cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, K, dist, rvecs, tvecs);
		}
	}

	list<Shape3D> result;
	if (out.hasChannel(channel_out_)) {
		result = out.get<list<Shape3D>>(channel_out_);
	}

	for (size_t i = 0; i < rvecs.size(); i++) {
		if (estimate_pose_) {
			auto &t = result.emplace_back();
			t.id = ids[i];
			t.type = ftl::codecs::Shape3DType::ARUCO;
			t.pose = (in.getPose() * matrix(rvecs[i], tvecs[i])).cast<float>();
			t.size = Eigen::Vector3f(1.0f, 1.0f, 0.0f)*marker_size_;
			t.label = "Aruco-" + std::to_string(ids[i]);
		}
	}

	out.create<list<Shape3D>>(channel_out_).list = result;
	return true;
}
