#ifndef _FTL_RECONSTRUCT_COMPACTORS_HPP_
#define _FTL_RECONSTRUCT_COMPACTORS_HPP_

#include <ftl/voxel_hash.hpp>

namespace ftl {
namespace cuda {

// Compact visible
void compactifyVisible(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const DepthCameraParams &camera, cudaStream_t);

// Compact allocated
void compactifyAllocated(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, cudaStream_t);

// Compact visible surfaces

}
}

#endif  // _FTL_RECONSTRUCT_COMPACTORS_HPP_
