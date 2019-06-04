#include "ftl/snapshot_source.hpp"

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

using namespace ftl::rgbd;

using std::string;

SnapshotSource::SnapshotSource(nlohmann::json &config, SnapshotReader &reader, const string &id) : RGBDSource(config) {
    Eigen::Matrix4f pose;
    reader.getCameraRGBD(id, rgb_, depth_, pose, params_);
    setPose(pose);
}
