// From: https://github.com/niessner/VoxelHashing/blob/master/DepthSensingCUDA/Source/CUDASceneRepHashSDF.h

#pragma once

#include <cuda_runtime.h>

#include <ftl/cuda_common.hpp>
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
	ftl::voxhash::DepthCamera gpu;
	DepthCameraParams params;
	cv::cuda::Stream stream;
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

	void setLastRigidTransform(const Eigen::Matrix4f& lastRigidTransform);


	const Eigen::Matrix4f getLastRigidTransform() const;

	/* Nick: To reduce weights between frames */
	void nextFrame();

	//! resets the hash to the initial state (i.e., clears all data)
	void reset();

	int cameraCount() const { return (int)cameras_.size(); }


	ftl::voxhash::HashData& getHashData() { return m_hashData; } 

	HashParams& getHashParams() { return m_hashParams; }

	unsigned int getOccupiedCount();

	//! debug only!
	void debugHash();

	cudaStream_t getIntegrationStream() const { return integ_stream_; }
	int getCUDADevice() const { return cuda_device_; }

	private:

	bool _initCUDA();
	HashParams _parametersFromConfig();
	void _create(const HashParams& params);
	void _destroy();
	void _alloc(int camid, cudaStream_t);
	void _compactifyVisible(const DepthCameraParams &camera);
	void _compactifyAllocated();
	//void _integrateDepthMap(const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams);
	void _integrateDepthMaps();
	void _garbageCollect();

	void _updateCameraConstant();

	HashParams		m_hashParams;
	ftl::voxhash::HashData		m_hashData;

	//CUDAScan		m_cudaScan;
	unsigned int	m_numIntegratedFrames;	//used for garbage collect
	unsigned int	m_frameCount;
	bool do_reset_;
	std::vector<Cameras> cameras_;
	cudaStream_t integ_stream_;
	bool reg_mode_;
	int cuda_device_;
};

};  // namespace voxhash
};  // namespace ftl
