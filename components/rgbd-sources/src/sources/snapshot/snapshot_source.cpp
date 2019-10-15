#include "snapshot_source.hpp"
#include "colour.hpp"
#include <loguru.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <vector>

using namespace ftl::rgbd;
using ftl::rgbd::detail::SnapshotSource;

using std::string;
using std::vector;

SnapshotSource::SnapshotSource(ftl::rgbd::Source *host, Snapshot &snapshot, const string &id) : detail::Source(host) {
	snapshot_ = snapshot;
	camera_idx_ = std::atoi(id.c_str());
	frame_idx_ = 0;

	Eigen::Matrix4d pose;
	snapshot.getPose(camera_idx_, pose);
	params_ = snapshot.getParameters(camera_idx_);

	/*
	ftl::rgbd::colourCorrection(rgb_, host->value("gamma", 1.0f), host->value("temperature", 6500));
	host->on("gamma", [this,host](const ftl::config::Event&) {
		ftl::rgbd::colourCorrection(rgb_, host->value("gamma", 1.0f), host->value("temperature", 6500));
	});
	*/

	// Add calibration to config object
	host_->getConfig()["focal"] = params_.fx;
	host_->getConfig()["centre_x"] = params_.cx;
	host_->getConfig()["centre_y"] = params_.cy;
	host_->getConfig()["baseline"] = params_.baseline;

	host_->on("focal", [this](const ftl::config::Event &e) {
		params_.fx = host_->value("focal", params_.fx);
		params_.fy = params_.fx;
	});

	host_->on("centre_x", [this](const ftl::config::Event &e) {
		params_.cx = host_->value("centre_x", params_.cx);
	});

	host_->on("centre_y", [this](const ftl::config::Event &e) {
		params_.cy = host_->value("centre_y", params_.cy);
	});

	LOG(INFO) << "POSE = " << pose;

    host->setPose(pose);

	mspf_ = 1000 / host_->value("fps", 20);
}

bool SnapshotSource::compute(int n, int b) {
	snapshot_.getLeftRGB(camera_idx_, frame_idx_, snap_rgb_);
	snapshot_.getLeftDepth(camera_idx_, frame_idx_, snap_depth_);

	snap_rgb_.copyTo(rgb_);
	snap_depth_.copyTo(depth_);

	auto cb = host_->callback();
	if (cb) cb(timestamp_, rgb_, depth_);

	frame_idx_ = (frame_idx_ + 1) % snapshot_.getFramesCount();

	return true;
}
