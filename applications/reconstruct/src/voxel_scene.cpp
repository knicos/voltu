#include <ftl/voxel_scene.hpp>
#include "compactors.hpp"
#include "garbage.hpp"
#include "integrators.hpp"
#include "depth_camera_cuda.hpp"

#include <opencv2/core/cuda_stream_accessor.hpp>

#include <vector>

using namespace ftl::voxhash;
using ftl::rgbd::Source;
using ftl::Configurable;
using cv::Mat;
using std::vector;

#define 	SAFE_DELETE_ARRAY(a)   { delete [] (a); (a) = NULL; }

extern "C" void resetCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
extern "C" void resetHashBucketMutexCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, cudaStream_t);
extern "C" void allocCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, int camid, const DepthCameraParams &depthCameraParams, cudaStream_t);
//extern "C" void fillDecisionArrayCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const DepthCameraData& depthCameraData);
//extern "C" void compactifyHashCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
//extern "C" unsigned int compactifyHashAllInOneCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams);
//extern "C" void integrateDepthMapCUDA(ftl::voxhash::HashData& hashData, const ftl::voxhash::HashParams& hashParams, const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, cudaStream_t);
//extern "C" void bindInputDepthColorTextures(const DepthCameraData& depthCameraData);


SceneRep::SceneRep(nlohmann::json &config) : Configurable(config), m_frameCount(0), do_reset_(false) {
	_initCUDA();

	// Allocates voxel structure on GPU
	_create(_parametersFromConfig());

	on("SDFVoxelSize", [this](const ftl::config::Event &e) {
		do_reset_ = true;
	});
	on("hashNumSDFBlocks", [this](const ftl::config::Event &e) {
		do_reset_ = true;
	});
	on("hashNumBuckets", [this](const ftl::config::Event &e) {
		do_reset_ = true;
	});
	on("hashMaxCollisionLinkedListSize", [this](const ftl::config::Event &e) {
		do_reset_ = true;
	});
	on("SDFTruncation", [this](const ftl::config::Event &e) {
		m_hashParams.m_truncation = value("SDFTruncation", 0.1f);
	});
	on("SDFTruncationScale", [this](const ftl::config::Event &e) {
		m_hashParams.m_truncScale = value("SDFTruncationScale", 0.01f);
	});
	on("SDFMaxIntegrationDistance", [this](const ftl::config::Event &e) {
		m_hashParams.m_maxIntegrationDistance = value("SDFMaxIntegrationDistance", 10.0f);
	});
	on("showRegistration", [this](const ftl::config::Event &e) {
		reg_mode_ = value("showRegistration", false);
	});

	reg_mode_ = value("showRegistration", false);

	cudaSafeCall(cudaStreamCreate(&integ_stream_));
	//integ_stream_ = 0;
}

SceneRep::~SceneRep() {
	_destroy();
	cudaStreamDestroy(integ_stream_);
}

bool SceneRep::_initCUDA() {
	// Do an initial CUDA check
	int cuda_device_count = 0;
	cudaSafeCall(cudaGetDeviceCount(&cuda_device_count));
	CHECK_GE(cuda_device_count, 1) << "No CUDA devices found";

	LOG(INFO) << "CUDA Devices (" << cuda_device_count << "):";

	vector<cudaDeviceProp> properties(cuda_device_count);
	for (int i=0; i<cuda_device_count; i++) {
		cudaSafeCall(cudaGetDeviceProperties(&properties[i], i));
		LOG(INFO) << " - " << properties[i].name;
	}

	int desired_device = value("cudaDevice", 0);
	cuda_device_ = (desired_device < cuda_device_count) ? desired_device : cuda_device_count-1;
	cudaSafeCall(cudaSetDevice(cuda_device_));

	// TODO:(Nick) Check memory is sufficient
	// TODO:(Nick) Find out what our compute capability should be.

	LOG(INFO) << "CUDA Compute: " << properties[cuda_device_].major << "." << properties[cuda_device_].minor;

	return true;
}

void SceneRep::addSource(ftl::rgbd::Source *src) {
	auto &cam = cameras_.emplace_back();
	cam.source = src;
	cam.params.m_imageWidth = 0;
}

extern "C" void updateCUDACameraConstant(ftl::voxhash::DepthCameraCUDA *data, int count);

void SceneRep::_updateCameraConstant() {
	std::vector<ftl::voxhash::DepthCameraCUDA> cams(cameras_.size());
	for (size_t i=0; i<cameras_.size(); ++i) {
		cams[i] = cameras_[i].gpu.data;
		cams[i].pose = MatrixConversion::toCUDA(cameras_[i].source->getPose().cast<float>());
		cams[i].poseInverse = MatrixConversion::toCUDA(cameras_[i].source->getPose().cast<float>().inverse());
	}
	updateCUDACameraConstant(cams.data(), cams.size());
}

int SceneRep::upload() {
	int active = 0;

	for (size_t i=0; i<cameras_.size(); ++i) {
		cameras_[i].source->grab();
	}

	for (size_t i=0; i<cameras_.size(); ++i) {
		auto &cam = cameras_[i];

		if (!cam.source->isReady()) {
			cam.params.m_imageWidth = 0;
			// TODO(Nick) : Free gpu allocs if was ready before
			LOG(INFO) << "Source not ready: " << cam.source->getURI();
			continue;
		} else {
			auto in = cam.source;

			cam.params.fx = in->parameters().fx;
			cam.params.fy = in->parameters().fy;
			cam.params.mx = -in->parameters().cx;
			cam.params.my = -in->parameters().cy;

			// Only now do we have camera parameters for allocations...
			if (cam.params.m_imageWidth == 0) {
				cam.params.m_imageWidth = in->parameters().width;
				cam.params.m_imageHeight = in->parameters().height;
				cam.params.m_sensorDepthWorldMax = in->parameters().maxDepth;
				cam.params.m_sensorDepthWorldMin = in->parameters().minDepth;
				cam.gpu.alloc(cam.params, true);
				LOG(INFO) << "GPU Allocated camera " << i;
			}
		}

		cam.params.flags = m_frameCount;
	}

	_updateCameraConstant();
	//cudaSafeCall(cudaDeviceSynchronize());

	for (size_t i=0; i<cameras_.size(); ++i) {
		auto &cam = cameras_[i];

		// Get the RGB-Depth frame from input
		Source *input = cam.source;
		Mat rgb, depth;

		// TODO(Nick) Direct GPU upload to save copy
		input->getFrames(rgb,depth);
		
		active += 1;

		if (depth.cols == 0) continue;

		// Must be in RGBA for GPU
		Mat rgbt, rgba;
		cv::cvtColor(rgb,rgbt, cv::COLOR_BGR2Lab);
		cv::cvtColor(rgbt,rgba, cv::COLOR_BGR2BGRA);

		// Send to GPU and merge view into scene
		//cam.gpu.updateParams(cam.params);
		cam.gpu.updateData(depth, rgba, cam.stream);

		//setLastRigidTransform(input->getPose().cast<float>());

		//make the rigid transform available on the GPU
		//m_hashData.updateParams(m_hashParams, cv::cuda::StreamAccessor::getStream(cam.stream));

		//if (i > 0) cudaSafeCall(cudaStreamSynchronize(cv::cuda::StreamAccessor::getStream(cameras_[i-1].stream)));

		//allocate all hash blocks which are corresponding to depth map entries
		if (value("voxels", false)) _alloc(i, cv::cuda::StreamAccessor::getStream(cam.stream));

		// Calculate normals
	}

	// Must have finished all allocations and rendering before next integration
	cudaSafeCall(cudaDeviceSynchronize());

	return active;
}

void SceneRep::integrate() {
	/*for (size_t i=0; i<cameras_.size(); ++i) {
		auto &cam = cameras_[i];

		setLastRigidTransform(cam.source->getPose().cast<float>());
		//m_hashData.updateParams(m_hashParams);

		//generate a linear hash array with only occupied entries
		_compactifyVisible(cam.params);

		//volumetrically integrate the depth data into the depth SDFBlocks
		//_integrateDepthMap(cam.gpu, cam.params);

		//_garbageCollect();

		m_numIntegratedFrames++;
	}*/

	_compactifyAllocated();
	_integrateDepthMaps();
}

void SceneRep::garbage() {
	//_compactifyAllocated();
	if (value("voxels", false)) _garbageCollect();

	//cudaSafeCall(cudaStreamSynchronize(integ_stream_));
}

/*void SceneRep::integrate(const Eigen::Matrix4f& lastRigidTransform, const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams, unsigned int* d_bitMask) {
		
	setLastRigidTransform(lastRigidTransform);

	//make the rigid transform available on the GPU
	m_hashData.updateParams(m_hashParams);

	//allocate all hash blocks which are corresponding to depth map entries
	_alloc(depthCameraData, depthCameraParams, d_bitMask);

	//generate a linear hash array with only occupied entries
	_compactifyHashEntries();

	//volumetrically integrate the depth data into the depth SDFBlocks
	_integrateDepthMap(depthCameraData, depthCameraParams);

	_garbageCollect(depthCameraData);

	m_numIntegratedFrames++;
}*/

/*void SceneRep::setLastRigidTransformAndCompactify(const Eigen::Matrix4f& lastRigidTransform, const DepthCameraData& depthCameraData) {
	setLastRigidTransform(lastRigidTransform);
	_compactifyHashEntries();
}*/

/* Nick: To reduce weights between frames */
void SceneRep::nextFrame() {
	if (do_reset_) {
		do_reset_ = false;
		_destroy();
		_create(_parametersFromConfig());
	} else {
		//ftl::cuda::compactifyAllocated(m_hashData, m_hashParams, integ_stream_);
		//if (reg_mode_) ftl::cuda::clearVoxels(m_hashData, m_hashParams); 
		//else ftl::cuda::starveVoxels(m_hashData, m_hashParams, integ_stream_);
		m_numIntegratedFrames = 0;
	}
}

//! resets the hash to the initial state (i.e., clears all data)
void SceneRep::reset() {
	m_numIntegratedFrames = 0;
	m_hashData.updateParams(m_hashParams);
	resetCUDA(m_hashData, m_hashParams);
}

unsigned int SceneRep::getOccupiedCount() {
	unsigned int count;
	cudaSafeCall(cudaMemcpy(&count, m_hashData.d_hashCompactifiedCounter, sizeof(unsigned int), cudaMemcpyDeviceToHost));
	return count+1;	//there is one more free than the address suggests (0 would be also a valid address)
}

HashParams SceneRep::_parametersFromConfig() {
	//auto &cfg = ftl::config::resolve(config);
	HashParams params;
	// First camera view is set to identity pose to be at the centre of
	// the virtual coordinate space.
	params.m_hashNumBuckets = value("hashNumBuckets", 100000);
	params.m_virtualVoxelSize = value("SDFVoxelSize", 0.006f);
	params.m_maxIntegrationDistance = value("SDFMaxIntegrationDistance", 10.0f);
	params.m_truncation = value("SDFTruncation", 0.1f);
	params.m_truncScale = value("SDFTruncationScale", 0.01f);
	params.m_integrationWeightSample = value("SDFIntegrationWeightSample", 10);
	params.m_integrationWeightMax = value("SDFIntegrationWeightMax", 255);
	params.m_spatialSmoothing = value("spatialSmoothing", 0.04f); // 4cm
	params.m_colourSmoothing = value("colourSmoothing", 20.0f);
	params.m_confidenceThresh = value("confidenceThreshold", 20.0f);
	params.m_flags = 0;
	params.m_flags |= (value("clipping", false)) ? ftl::voxhash::kFlagClipping : 0;
	params.m_flags |= (value("mls", false)) ? ftl::voxhash::kFlagMLS : 0;
	params.m_maxBounds = make_int3(
		value("bbox_x_max", 2.0f) / (params.m_virtualVoxelSize*SDF_BLOCK_SIZE),
		value("bbox_y_max", 2.0f) / (params.m_virtualVoxelSize*SDF_BLOCK_SIZE),
		value("bbox_z_max", 2.0f) / (params.m_virtualVoxelSize*SDF_BLOCK_SIZE));
	params.m_minBounds = make_int3(
		value("bbox_x_min", -2.0f) / (params.m_virtualVoxelSize*SDF_BLOCK_SIZE),
		value("bbox_y_min", -2.0f) / (params.m_virtualVoxelSize*SDF_BLOCK_SIZE),
		value("bbox_z_min", -2.0f) / (params.m_virtualVoxelSize*SDF_BLOCK_SIZE));
	return params;
}

void SceneRep::_create(const HashParams& params) {
	m_hashParams = params;
	m_hashData.allocate(m_hashParams);

	reset();
}

void SceneRep::_destroy() {
	m_hashData.free();
}

void SceneRep::_alloc(int camid, cudaStream_t stream) {
	// NOTE (nick): We might want this later...
	/*if (false) {
		// TODO(Nick) Make this work without memcpy to host first
		allocate until all blocks are allocated
		unsigned int prevFree = 0; //getHeapFreeCount();
		while (1) {
			resetHashBucketMutexCUDA(m_hashData, m_hashParams, stream);
			allocCUDA(m_hashData, m_hashParams, camid, cameras_[camid].params, stream);

			unsigned int currFree = getHeapFreeCount();

			if (prevFree != currFree) {
				prevFree = currFree;
			}
			else {
				break;
			}
		}
	}
	else {*/
		//this version is faster, but it doesn't guarantee that all blocks are allocated (staggers alloc to the next frame)
		resetHashBucketMutexCUDA(m_hashData, m_hashParams, stream);
		allocCUDA(m_hashData, m_hashParams, camid, cameras_[camid].params, stream);
	//}
}


void SceneRep::_compactifyVisible(const DepthCameraParams &camera) { //const DepthCameraData& depthCameraData) {
	ftl::cuda::compactifyOccupied(m_hashData, m_hashParams, integ_stream_);		//this version uses atomics over prefix sums, which has a much better performance
	//m_hashData.updateParams(m_hashParams);	//make sure numOccupiedBlocks is updated on the GPU
}

void SceneRep::_compactifyAllocated() {
	ftl::cuda::compactifyAllocated(m_hashData, m_hashParams, integ_stream_);		//this version uses atomics over prefix sums, which has a much better performance
	//std::cout << "Occ blocks = " << m_hashParams.m_numOccupiedBlocks << std::endl;
	//m_hashData.updateParams(m_hashParams);	//make sure numOccupiedBlocks is updated on the GPU
}

/*void SceneRep::_integrateDepthMap(const DepthCameraData& depthCameraData, const DepthCameraParams& depthCameraParams) {
	if (!reg_mode_) ftl::cuda::integrateDepthMap(m_hashData, m_hashParams, depthCameraData, depthCameraParams, integ_stream_);
	else ftl::cuda::integrateRegistration(m_hashData, m_hashParams, depthCameraData, depthCameraParams, integ_stream_);
}*/

extern "C" void bilateralFilterFloatMap(float* d_output, float* d_input, float sigmaD, float sigmaR, unsigned int width, unsigned int height);

void SceneRep::_integrateDepthMaps() {
	//cudaSafeCall(cudaDeviceSynchronize());

	for (size_t i=0; i<cameras_.size(); ++i) {
		//ftl::cuda::clear_depth(*(cameras_[i].gpu.depth2_tex_), integ_stream_);
		ftl::cuda::clear_points(*(cameras_[i].gpu.points_tex_), integ_stream_);
		ftl::cuda::mls_smooth(*(cameras_[i].gpu.points_tex_), m_hashParams, cameras_.size(), i, integ_stream_);
		//ftl::cuda::int_to_float(*(cameras_[i].gpu.depth2_tex_), *(cameras_[i].gpu.depth_tex_), 1.0f / 1000.0f, integ_stream_);
		//ftl::cuda::hole_fill(*(cameras_[i].gpu.depth2_tex_), *(cameras_[i].gpu.depth_tex_), cameras_[i].params, integ_stream_);
		//bilateralFilterFloatMap(cameras_[i].gpu.depth_tex_->devicePtr(), cameras_[i].gpu.depth3_tex_->devicePtr(), 3, 7, cameras_[i].gpu.depth_tex_->width(), cameras_[i].gpu.depth_tex_->height());
	}
	if (value("voxels", false)) ftl::cuda::integrateDepthMaps(m_hashData, m_hashParams, cameras_.size(), integ_stream_);
}

void SceneRep::_garbageCollect() {
	//ftl::cuda::garbageCollectIdentify(m_hashData, m_hashParams, integ_stream_);
	resetHashBucketMutexCUDA(m_hashData, m_hashParams, integ_stream_);	//needed if linked lists are enabled -> for memeory deletion
	ftl::cuda::garbageCollectFree(m_hashData, m_hashParams, integ_stream_);
}
