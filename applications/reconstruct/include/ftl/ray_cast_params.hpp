#include <ftl/cuda_util.hpp>

#include <ftl/cuda_matrix_util.hpp>

struct __align__(16) RayCastParams {
	float4x4 m_viewMatrix;
	float4x4 m_viewMatrixInverse;
	float4x4 m_intrinsics;
	float4x4 m_intrinsicsInverse;

	unsigned int m_width;
	unsigned int m_height;

	unsigned int m_numOccupiedSDFBlocks;
	unsigned int m_maxNumVertices;
	int m_splatMinimum;

	float m_minDepth;
	float m_maxDepth;
	float m_rayIncrement;
	float m_thresSampleDist;
	float m_thresDist;
	bool  m_useGradients;

	uint dummy0;
};
