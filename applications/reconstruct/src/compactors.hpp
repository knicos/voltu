#ifndef _FTL_RECONSTRUCT_COMPACTORS_HPP_
#define _FTL_RECONSTRUCT_COMPACTORS_HPP_

#include <ftl/voxel_hash.hpp>

namespace ftl {
namespace cuda {

// Compact visible
unsigned int compactifyVisible(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);

// Compact allocated
unsigned int compactifyAllocated(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);

// Compact visible surfaces

}
}

#endif  // _FTL_RECONSTRUCT_COMPACTORS_HPP_
