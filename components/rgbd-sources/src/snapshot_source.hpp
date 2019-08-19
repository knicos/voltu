#pragma once
#ifndef _FTL_RGBD_SNAPSHOT_SOURCE_HPP_
#define _FTL_RGBD_SNAPSHOT_SOURCE_HPP_

#include <loguru.hpp>

#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/snapshot.hpp>

namespace ftl {
namespace rgbd {
namespace detail {

class SnapshotSource : public detail::Source {
	public:
	SnapshotSource(ftl::rgbd::Source *);
	SnapshotSource(ftl::rgbd::Source *, ftl::rgbd::Snapshot &snapshot, const std::string &id);
	~SnapshotSource() {};

	bool capture(int64_t ts) { timestamp_ = ts; return true; }
	bool retrieve() { return true; }
	bool compute(int n, int b);
	bool isReady() { return true; }

	//void reset();
	private:
	size_t frame_idx_;
	size_t camera_idx_;
	
	ftl::rgbd::Snapshot snapshot_;

	cv::Mat snap_rgb_;
	cv::Mat snap_depth_;
	int mspf_;
};

}
}
}

#endif  // _FTL_RGBD_SNAPSHOT_SOURCE_HPP_
