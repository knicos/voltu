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
		auto &cfg = ftl::config::resolve(config);
		create(parametersFromConfig(cfg));
		hash_render_ = value("hash_renderer", false);

		on("hash_renderer", [this](const ftl::config::Event &e) {
			hash_render_ = value("hash_renderer", false);
		});

		on("width", [this](const ftl::config::Event &e) {
			m_params.m_width = value("width", 640);
		});

		on("height", [this](const ftl::config::Event &e) {
			m_params.m_height = value("height", 480);
		});

		on("width", [this](const ftl::config::Event &e) {
			m_params.m_width = value("width", 640);
		});

		on("max_depth", [this](const ftl::config::Event &e) {
			m_params.m_maxDepth = value("max_depth", 20.0f);
		});

		on("min_depth", [this](const ftl::config::Event &e) {
			m_params.m_minDepth = value("min_depth", 20.0f);
		});

		on("showBlockBorders", [this](const ftl::config::Event &e) {
			if (value("showBlockBorders", false)) m_params.m_flags |= kShowBlockBorders;
			else m_params.m_flags &= ~kShowBlockBorders;
		});
	}

	bool isIntegerDepth() const { return hash_render_; }

	~CUDARayCastSDF(void) {
		destroy();
	}

	static RayCastParams parametersFromConfig(const nlohmann::json& gas) {
		RayCastParams params;
		params.m_width = gas["width"].get<unsigned int>();
		params.m_height = gas["height"].get<unsigned int>();
		params.m_intrinsics = MatrixConversion::toCUDA(Eigen::Matrix4f());
		params.m_intrinsicsInverse = MatrixConversion::toCUDA(Eigen::Matrix4f());
		params.m_minDepth = gas["min_depth"].get<float>();
		params.m_maxDepth = gas["max_depth"].get<float>();
		params.m_rayIncrement = gas["SDFRayIncrementFactor"].get<float>() * gas["SDFTruncation"].get<float>();
		params.m_thresSampleDist = gas["SDFRayThresSampleDistFactor"].get<float>() * params.m_rayIncrement;
		params.m_thresDist = gas["SDFRayThresDistFactor"].get<float>() * params.m_rayIncrement;
		params.m_useGradients = gas["SDFUseGradients"].get<bool>();

		uint flags = 0;
		if (gas.value("showBlockBorders", false)) flags |= kShowBlockBorders;
		params.m_flags = flags;


		return params;
	}

	void render(ftl::voxhash::HashData& hashData, ftl::voxhash::HashParams& hashParams, const DepthCameraParams& cameraParams, const Eigen::Matrix4f& lastRigidTransform);

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

	void compactifyHashEntries(ftl::voxhash::HashData& hashData, ftl::voxhash::HashParams& hashParams);

	void rayIntervalSplatting(const ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const Eigen::Matrix4f& lastRigidTransform); // rasterize

	RayCastParams m_params;
	RayCastData m_data;
	bool hash_render_;

	// DX11RayIntervalSplatting m_rayIntervalSplatting;
};
