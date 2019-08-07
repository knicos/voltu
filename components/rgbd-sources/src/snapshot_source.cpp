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

SnapshotSource::SnapshotSource(ftl::rgbd::Source *host, SnapshotReader &reader, const string &id) : detail::Source(host) {
    Eigen::Matrix4d pose;
    reader.getCameraRGBD(id, snap_rgb_, snap_depth_, pose, params_);

	rgb_ = snap_rgb_;
	depth_ = snap_depth_;

	if (rgb_.empty()) LOG(ERROR) << "Did not load snapshot rgb - " << id;
	if (depth_.empty()) LOG(ERROR) << "Did not load snapshot depth - " << id;
	if (params_.width != rgb_.cols) LOG(ERROR) << "Camera parameters corrupt for " << id;

	ftl::rgbd::colourCorrection(rgb_, host->value("gamma", 1.0f), host->value("temperature", 6500));

	host->on("gamma", [this,host](const ftl::config::Event&) {
		ftl::rgbd::colourCorrection(rgb_, host->value("gamma", 1.0f), host->value("temperature", 6500));
	});

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
}

bool SnapshotSource::compute(int n, int b) {
	snap_rgb_.copyTo(rgb_);
	snap_depth_.copyTo(depth_);
	return true;
}
