//#include <stdafx.h>

#include <ftl/voxel_hash.hpp>
#include "compactors.hpp"

//#include "Util.h"

#include <ftl/ray_cast_sdf.hpp>


extern "C" void renderCS(
	const ftl::voxhash::HashData& hashData,
	const RayCastData &rayCastData,
	const RayCastParams &rayCastParams);

extern "C" void computeNormals(float4* d_output, float3* d_input, unsigned int width, unsigned int height);
extern "C" void convertDepthFloatToCameraSpaceFloat3(float3* d_output, float* d_input, float4x4 intrinsicsInv, unsigned int width, unsigned int height, const DepthCameraData& depthCameraData);

extern "C" void resetRayIntervalSplatCUDA(RayCastData& data, const RayCastParams& params);
extern "C" void rayIntervalSplatCUDA(const ftl::voxhash::HashData& hashData,
								 const RayCastData &rayCastData, const RayCastParams &rayCastParams);

extern "C" void nickRenderCUDA(const ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const RayCastData &rayCastData, const RayCastParams &params);



void CUDARayCastSDF::create(const RayCastParams& params)
{
	m_params = params;
	m_data.allocate(m_params);
	//m_rayIntervalSplatting.OnD3D11CreateDevice(DXUTGetD3D11Device(), params.m_width, params.m_height);
}

void CUDARayCastSDF::destroy(void)
{
	m_data.free();
	//m_rayIntervalSplatting.OnD3D11DestroyDevice();
}

//extern "C" unsigned int compactifyHashAllInOneCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);


void CUDARayCastSDF::compactifyHashEntries(ftl::voxhash::HashData& hashData, ftl::voxhash::HashParams& hashParams) { //const DepthCameraData& depthCameraData) {
		
	hashParams.m_numOccupiedBlocks = ftl::cuda::compactifyVisible(hashData, hashParams);		//this version uses atomics over prefix sums, which has a much better performance
	std::cout << "Ray blocks = " << hashParams.m_numOccupiedBlocks << std::endl;
	hashData.updateParams(hashParams);	//make sure numOccupiedBlocks is updated on the GPU
}

void CUDARayCastSDF::render(ftl::voxhash::HashData& hashData, ftl::voxhash::HashParams& hashParams, const DepthCameraParams& cameraParams, const Eigen::Matrix4f& lastRigidTransform)
{
	updateConstantDepthCameraParams(cameraParams);
	//rayIntervalSplatting(hashData, hashParams, lastRigidTransform);
	//m_data.d_rayIntervalSplatMinArray = m_rayIntervalSplatting.mapMinToCuda();
	//m_data.d_rayIntervalSplatMaxArray = m_rayIntervalSplatting.mapMaxToCuda();

	m_params.m_numOccupiedSDFBlocks = hashParams.m_numOccupiedBlocks;
	m_params.m_viewMatrix = MatrixConversion::toCUDA(lastRigidTransform.inverse());
	m_params.m_viewMatrixInverse = MatrixConversion::toCUDA(lastRigidTransform);
	m_data.updateParams(m_params);

	compactifyHashEntries(hashData, hashParams);

	if (hash_render_) nickRenderCUDA(hashData, hashParams, m_data, m_params);
	else renderCS(hashData, m_data, m_params);

	//convertToCameraSpace(cameraData);
	if (!m_params.m_useGradients)
	{
		computeNormals(m_data.d_normals, m_data.d_depth3, m_params.m_width, m_params.m_height);
	}

	//m_rayIntervalSplatting.unmapCuda();

}


void CUDARayCastSDF::convertToCameraSpace(const DepthCameraData& cameraData)
{
	convertDepthFloatToCameraSpaceFloat3(m_data.d_depth3, m_data.d_depth, m_params.m_intrinsicsInverse, m_params.m_width, m_params.m_height, cameraData);
	
	if(!m_params.m_useGradients) {
		computeNormals(m_data.d_normals, m_data.d_depth3, m_params.m_width, m_params.m_height);
	}
}

void CUDARayCastSDF::rayIntervalSplatting(const ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const Eigen::Matrix4f& lastRigidTransform)
{
	if (hashParams.m_numOccupiedBlocks == 0)	return;

	if (m_params.m_maxNumVertices <= 6*hashParams.m_numOccupiedBlocks) { // 6 verts (2 triangles) per block
		throw std::runtime_error("not enough space for vertex buffer for ray interval splatting");
	}

	m_params.m_numOccupiedSDFBlocks = hashParams.m_numOccupiedBlocks;
	m_params.m_viewMatrix = MatrixConversion::toCUDA(lastRigidTransform.inverse());
	m_params.m_viewMatrixInverse = MatrixConversion::toCUDA(lastRigidTransform);

	m_data.updateParams(m_params); // !!! debugging

	//don't use ray interval splatting (cf CUDARayCastSDF.cu -> line 40
	//m_rayIntervalSplatting.rayIntervalSplatting(DXUTGetD3D11DeviceContext(), hashData, cameraData, m_data, m_params, m_params.m_numOccupiedSDFBlocks*6);
}