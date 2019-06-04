#pragma once
#ifndef _FTL_RGBD_SNAPSHOT_SOURCE_HPP_
#define _FTL_RGBD_SNAPSHOT_SOURCE_HPP_

#include <glog/logging.h>

#include "ftl/rgbd_source.hpp"
#include "ftl/snapshot.hpp"

namespace ftl {
namespace rgbd {

class SnapshotSource : public RGBDSource {
	public:
	SnapshotSource(nlohmann::json &config, ftl::rgbd::SnapshotReader &reader, const std::string &id);
	~SnapshotSource() {};
	void grab() override {};
};

};
};

#endif  // _FTL_RGBD_SNAPSHOT_SOURCE_HPP_
