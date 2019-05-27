#pragma once

#include <ftl/configurable.hpp>
#include <ftl/matrix_conversion.hpp>
#include <ftl/cuda_matrix_util.hpp>
#include <ftl/depth_camera.hpp>
#include <ftl/ray_cast_util.hpp>
#include <nlohmann/json.hpp>

// #include "DX11RayIntervalSplatting.h"

class CUDARayCastSDF : public ftl::Configurable
{
public:
	CUDARayCastSDF(nlohmann::json& config) : ftl::Configurable(config) {
		create(parametersFromConfig(config));
		hash_render_ = config.value("hash_renderer", false);
	}

	~CUDARayCastSDF(void) {
		destroy();
	}

	static RayCastParams parametersFromConfig(const nlohmann::json& gas) {
		RayCastParams params;
		params.m_width = gas["adapterWidth"].get<unsigned int>();
		params.m_height = gas["adapterHeight"].get<unsigned int>();
		params.m_intrinsics = MatrixConversion::toCUDA(Eigen::Matrix4f());
		params.m_intrinsicsInverse = MatrixConversion::toCUDA(Eigen::Matrix4f());
		params.m_minDepth = gas["sensorDepthMin"].get<float>();
		params.m_maxDepth = gas["sensorDepthMax"].get<float>();
		params.m_rayIncrement = gas["SDFRayIncrementFactor"].get<float>() * gas["SDFTruncation"].get<float>();
		params.m_thresSampleDist = gas["SDFRayThresSampleDistFactor"].get<float>() * params.m_rayIncrement;
		params.m_thresDist = gas["SDFRayThresDistFactor"].get<float>() * params.m_rayIncrement;
		params.m_useGradients = gas["SDFUseGradients"].get<bool>();

		params.m_maxNumVertices = gas["hashNumSDFBlocks"].get<unsigned int>() * 6;

		return params;
	}

	void render(const ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const DepthCameraData& cameraData, const Eigen::Matrix4f& lastRigidTransform);

	const RayCastData& getRayCastData(void) {
		return m_data;
	}
	const RayCastParams& getRayCastParams() const {
		return m_params;
	}


	// debugging
	void convertToCameraSpace(const DepthCameraData& cameraData);

private:

	void create(const RayCastParams& params);
	void destroy(void);

	void rayIntervalSplatting(const ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const DepthCameraData& cameraData, const Eigen::Matrix4f& lastRigidTransform); // rasterize

	RayCastParams m_params;
	RayCastData m_data;
	bool hash_render_;

	// DX11RayIntervalSplatting m_rayIntervalSplatting;
};
