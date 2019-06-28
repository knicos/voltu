// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/CUDASceneRepHashSDF.h

#pragma once

#include <cuda_runtime.h>

#include <ftl/rgbd/source.hpp>
#include <ftl/configurable.hpp>
#include <ftl/matrix_conversion.hpp>
#include <ftl/voxel_hash.hpp>
#include <ftl/depth_camera.hpp>
#include <unordered_set>

namespace ftl {
namespace voxhash {

struct Cameras {
	ftl::rgbd::Source *source;
	DepthCameraData gpu;
	DepthCameraParams params;
};

class SceneRep : public ftl::Configurable {
	public:
	SceneRep(nlohmann::json &config);
	~SceneRep();

	void addSource(ftl::rgbd::Source *);

	/**
	 * Send all camera frames to GPU and allocate required voxels.
	 */
	int upload();

	/**
	 * Merge all camera frames into the voxel hash datastructure.
	 */
	void integrate();

	/**
	 * Remove any voxel blocks that are no longer used.
	 */
	void garbage();

	// Mark voxels as surfaces
	// void isosurface();

	/**
	 * Note: lastRigidTransform appears to be the estimated camera pose.
	 * Note: bitMask can be nullptr if not streaming out voxels from GPU
	 */
	//void integrate(const Eigen::Matrix4f& lastRigidTransform, const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, unsigned int* d_bitMask);

	void setLastRigidTransform(const Eigen::Matrix4f& lastRigidTransform);

	//void setLastRigidTransformAndCompactify(const Eigen::Matrix4f& lastRigidTransform, const DepthCameraData& depthCameraData);


	const Eigen::Matrix4f getLastRigidTransform() const;

	/* Nick: To reduce weights between frames */
	void nextFrame();

	//! resets the hash to the initial state (i.e., clears all data)
	void reset();


	ftl::voxhash::HashData& getHashData() { return m_hashData; } 

	HashParams& getHashParams() { return m_hashParams; }

	//! debug only!
	unsigned int getHeapFreeCount();

	//! debug only!
	void debugHash();

	private:

	HashParams _parametersFromConfig();
	void _create(const HashParams& params);
	void _destroy();
	void _alloc(const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, const unsigned int* d_bitMask);
	void _compactifyVisible();
	void _compactifyAllocated();
	void _integrateDepthMap(const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams);
	void _garbageCollect();



	HashParams		m_hashParams;
	ftl::voxhash::HashData		m_hashData;

	//CUDAScan		m_cudaScan;
	unsigned int	m_numIntegratedFrames;	//used for garbage collect
	unsigned int	m_frameCount;
	bool do_reset_;
	std::vector<Cameras> cameras_;
};

};  // namespace voxhash
};  // namespace ftl
