#include "catch.hpp"

//---- Mocks -------------------------------------------------------------------

#include <ftl/rgbd/source.hpp>
#include <ftl/config.h>

#include <nlohmann/json.hpp>

static std::string last_type = "";

namespace ftl {
namespace rgbd {

class Snapshot {};

class SnapshotReader {
	public:
	explicit SnapshotReader(const std::string &) {}
	Snapshot readArchive() { return Snapshot(); };
};

class Player {
	public:
	explicit Player(std::istream &) {}

	bool begin() { return true; }
	bool end() { return true; }
};

namespace detail {

class ImageSource : public ftl::rgbd::detail::Source {
	public:
	explicit ImageSource(ftl::rgbd::Source *host) : ftl::rgbd::detail::Source(host) {
		last_type = "image";
	}
	ImageSource(ftl::rgbd::Source *host, const std::string &f) : ftl::rgbd::detail::Source(host) {
		last_type = "image";
	}

	bool capture(int64_t ts) { return true; }
	bool retrieve() { return true; }
	bool compute(int n, int b) { return true; };
	bool isReady() { return true; };
};

class StereoVideoSource : public ftl::rgbd::detail::Source {
	public:
	explicit StereoVideoSource(ftl::rgbd::Source *host) : ftl::rgbd::detail::Source(host) {
		last_type = "video";
	}
	StereoVideoSource(ftl::rgbd::Source *host, const std::string &f) : ftl::rgbd::detail::Source(host) {
		last_type = "video";
	}

	bool capture(int64_t ts) { return true; }
	bool retrieve() { return true; }
	bool compute(int n, int b) { return true; };
	bool isReady() { return true; };
};

class NetSource : public ftl::rgbd::detail::Source {
	public:
	explicit NetSource(ftl::rgbd::Source *host) : ftl::rgbd::detail::Source(host) {
		last_type = "net";
	}

	bool capture(int64_t ts) { return true; }
	bool retrieve() { return true; }
	bool compute(int n, int b) { return true; };
	bool isReady() { return true; };
};

class SnapshotSource : public ftl::rgbd::detail::Source {
	public:
	SnapshotSource(ftl::rgbd::Source *host, ftl::rgbd::Snapshot &r, const std::string &) : ftl::rgbd::detail::Source(host) {
		last_type = "snapshot";
	}

	bool capture(int64_t ts) { return true; }
	bool retrieve() { return true; }
	bool compute(int n, int b) { return true; };
	bool isReady() { return true; };
};

class FileSource : public ftl::rgbd::detail::Source {
	public:
	FileSource(ftl::rgbd::Source *host, ftl::rgbd::Player *r, int) : ftl::rgbd::detail::Source(host) {
		last_type = "filesource";
	}

	bool capture(int64_t ts) { return true; }
	bool retrieve() { return true; }
	bool compute(int n, int b) { return true; };
	bool isReady() { return true; };
};

class RealsenseSource : public ftl::rgbd::detail::Source {
	public:
	explicit RealsenseSource(ftl::rgbd::Source *host) : ftl::rgbd::detail::Source(host) {
		last_type = "realsense";
	}

	bool capture(int64_t ts) { return true; }
	bool retrieve() { return true; }
	bool compute(int n, int b) { return true; };
	bool isReady() { return true; };
};

class MiddleburySource : public ftl::rgbd::detail::Source {
	public:
	MiddleburySource(ftl::rgbd::Source *host, const std::string &dir) : ftl::rgbd::detail::Source(host) {
		last_type = "middlebury";
	}

	bool capture(int64_t ts) { return true; }
	bool retrieve() { return true; }
	bool compute(int n, int b) { return true; };
	bool isReady() { return true; };
};

}	
}	
}

//---- Sources -----------------------------------------------------------------

// Prevent these headers...
#define _FTL_RGBD_STEREOVIDEO_HPP_
#define _FTL_RGBD_NET_HPP_
#define _FTL_RGBD_SNAPSHOT_HPP_
#define _FTL_RGBD_SNAPSHOT_SOURCE_HPP_
#define _FTL_RGBD_IMAGE_HPP_
#define _FTL_RGBD_REALSENSE_HPP_
#define _FTL_RGBD_MIDDLEBURY_SOURCE_HPP_
#define _FTL_RGBD_FILE_SOURCE_HPP_

#include "../src/source.cpp"


//---- Tests -------------------------------------------------------------------

using ftl::rgbd::Source;
using ftl::config::json_t;

TEST_CASE("ftl::create<Source>(cfg)", "[rgbd]") {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	SECTION("with an invalid uri") {
		json_t cfg = json_t{
			{"$id","ftl://test/2"},
			{"uri","not a uri"}
		};

		Source *src = ftl::create<Source>(cfg);

		REQUIRE( src );
		REQUIRE( !src->isReady() );
	}

	
}

