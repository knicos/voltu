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
	SnapshotSource(ftl::rgbd::Source *, ftl::rgbd::SnapshotReader &reader, const std::string &id);
	~SnapshotSource() {};

	bool compute(int n, int b) override { return true; };
	bool isReady() { return true; }

	//void reset();

};

}
}
}

#endif  // _FTL_RGBD_SNAPSHOT_SOURCE_HPP_
