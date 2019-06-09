#include "snapshot_source.hpp"

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

using namespace ftl::rgbd;
using ftl::rgbd::detail::SnapshotSource;

using std::string;

SnapshotSource::SnapshotSource(ftl::rgbd::Source *host, SnapshotReader &reader, const string &id) : detail::Source(host) {
    Eigen::Matrix4f pose;
    reader.getCameraRGBD(id, rgb_, depth_, pose, params_);
    setPose(pose);
}
