#include "ftl/operators/detectandtrack.hpp"
#include "ftl/codecs/transformation.hpp"

using ftl::operators::ArUco;
using ftl::rgbd::Frame;

using ftl::codecs::Channel;
using ftl::codecs::Transformation;

using cv::Mat;
using cv::Point2f;
using cv::Vec3d;

using std::vector;

ArUco::ArUco(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {
	dictionary_ = cv::aruco::getPredefinedDictionary(cfg->value("dictionary", 0));
	params_ = cv::aruco::DetectorParameters::create();

	debug_ = cfg->value("debug", false);
	estimate_pose_ = cfg->value("estimate_pose", false);
	auto marker_size = cfg->get<float>("marker_size");
	if (!marker_size || (*marker_size < 0.0f)) {
		marker_size_ = 0.0;
		estimate_pose_ = false;
	}
	else {
		marker_size_ = *marker_size;
	}

	channel_in_ = Channel::Colour;
	channel_out_ = Channel::Transforms;
}

bool ArUco::apply(Frame &in, Frame &out, cudaStream_t stream) {
	if (!in.hasChannel(channel_in_)) { return false; }
	in.download(channel_in_);

	Mat im = in.get<Mat>(channel_in_);
	Mat K = in.getLeftCamera().getCameraMatrix();
	Mat dist = cv::Mat::zeros(cv::Size(5, 1), CV_64FC1);

	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> ids;

	cv::aruco::detectMarkers(	im, dictionary_,
								corners, ids, params_, cv::noArray(), K);

	std::vector<Vec3d> rvecs;
	std::vector<Vec3d> tvecs;

	if (estimate_pose_) {
		cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, K, dist, rvecs, tvecs);
	}

	vector<Transformation> result;
	for (size_t i = 0; i < rvecs.size(); i++) {
		if (estimate_pose_) {
			result.push_back(ftl::codecs::Transformation(ids[i], rvecs[i], tvecs[i]));
		}
	}

	out.create(channel_out_, result);

	if (debug_) {
		cv::aruco::drawDetectedMarkers(im, corners, ids);
		if (estimate_pose_) {
			for (size_t i = 0; i < rvecs.size(); i++) {
					cv::aruco::drawAxis(im, K, dist, rvecs[i], tvecs[i], marker_size_);
			}
		}
	}

	// TODO: should be uploaded by operator which requires data on GPU
	in.upload(channel_in_);

	return true;
}