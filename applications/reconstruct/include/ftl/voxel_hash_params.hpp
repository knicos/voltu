// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/CUDAHashParams.h

#pragma once

//#include <cutil_inline.h>
//#include <cutil_math.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace voxhash {

//TODO might have to be split into static and dynamics
struct __align__(16) HashParams {
	HashParams() {
	}

	float4x4		m_rigidTransform;
	float4x4		m_rigidTransformInverse;

	unsigned int	m_hashNumBuckets;
	unsigned int	m_deprecated1;
	unsigned int	m_deprecated2; //m_hashMaxCollisionLinkedListSize;
	unsigned int	m_numSDFBlocks;

	int				m_SDFBlockSize;
	float			m_virtualVoxelSize;
	unsigned int	m_numOccupiedBlocks;	//occupied blocks in the viewing frustum
	
	float			m_maxIntegrationDistance;
	float			m_truncScale;
	float			m_truncation;
	unsigned int	m_integrationWeightSample;
	unsigned int	m_integrationWeightMax;

	float3			m_streamingVoxelExtents;
	int3			m_streamingGridDimensions;
	int3			m_streamingMinGridPos;
	unsigned int	m_streamingInitialChunkListSize;
	uint2			m_dummy;

};

}  // namespace voxhash
}  // namespace ftl
