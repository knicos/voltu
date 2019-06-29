#ifndef _FTL_RECONSTRUCTION_GARBAGE_HPP_
#define _FTL_RECONSTRUCTION_GARBAGE_HPP_

namespace ftl {
namespace cuda {

void starveVoxels(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
void garbageCollectIdentify(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, cudaStream_t stream);
void garbageCollectFree(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, cudaStream_t stream);

}
}

#endif  // _FTL_RECONSTRUCTION_GARBAGE_HPP_
