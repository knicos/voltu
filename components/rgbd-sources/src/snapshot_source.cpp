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
    reader.getCameraRGBD(id, rgb_, depth_, pose, params_);

	ftl::rgbd::colourCorrection(rgb_, host->value("gamma", 1.0f), host->value("temperature", 6500));

    setPose(pose);
}
