#ifndef _FTL_POINT_CLOUD_HPP_
#define _FTL_POINT_CLOUD_HPP_

namespace ftl {
namespace pointcloud {


struct VoxelPoint {
	union {
	uint64_t val;
	struct {
	uint16_t x : 12;  // Block X
	uint16_t y : 12;  // Block Y
	uint16_t z : 12;  // Block Z
	uint16_t v : 9;   // Voxel offset in block 0-511
	};
	};
};

struct VoxelColour {
	union {
	uint32_t val;
	struct {
	uint8_t b;
	uint8_t g;
	uint8_t r;
	uint8_t a;
	};
	};
};


}
}

#endif  // _FTL_POINT_CLOUD_HPP_
