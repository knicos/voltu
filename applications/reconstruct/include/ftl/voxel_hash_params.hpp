// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/CUDAHashParams.h

#pragma once

//#include <cutil_inline.h>
//#include <cutil_math.h>
#include <vector_types.h>
#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>

namespace ftl {
namespace voxhash {

static const unsigned int kFlagClipping = 0x00000001;
static const unsigned int kFlagMLS = 0x00000002;

//TODO might have to be split into static and dynamics
struct __align__(16) HashParams {
	HashParams() {
	}

	unsigned int	m_hashNumBuckets;
	float			m_virtualVoxelSize;
	float			m_maxIntegrationDistance;
	float			m_truncScale;
	float			m_truncation;
	unsigned int	m_integrationWeightSample;
	unsigned int	m_integrationWeightMax;

	float3 m_minBounds;
	float3 m_maxBounds;
	float m_spatialSmoothing;
	float m_colourSmoothing;
	float m_confidenceThresh;

	unsigned int m_flags;
};

}  // namespace voxhash
}  // namespace ftl
