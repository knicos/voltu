#include <ftl/cuda_common.hpp>
#include <ftl/cuda_util.hpp>
#include <ftl/depth_camera.hpp>
#include "depth_camera_cuda.hpp"

#include "mls_cuda.hpp"

#define T_PER_BLOCK 16
#define MINF __int_as_float(0xff800000)

using ftl::voxhash::DepthCameraCUDA;
using ftl::voxhash::HashData;
using ftl::voxhash::HashParams;
using ftl::cuda::TextureObject;
using ftl::render::SplatParams;

extern __constant__ ftl::voxhash::DepthCameraCUDA c_cameras[MAX_CAMERAS];
extern __constant__ HashParams c_hashParams;

/// ===== MLS Smooth

/*
 * Kim, K., Chalidabhongse, T. H., Harwood, D., & Davis, L. (2005).
 * Real-time foreground-background segmentation using codebook model.
 * Real-Time Imaging. https://doi.org/10.1016/j.rti.2004.12.004
 */
 __device__ float colordiffFloat(const uchar4 &pa, const uchar4 &pb) {
	const float x_2 = pb.x * pb.x + pb.y * pb.y + pb.z * pb.z;
	const float v_2 = pa.x * pa.x + pa.y * pa.y + pa.z * pa.z;
	const float xv_2 = powf(float(pb.x * pa.x + pb.y * pa.y + pb.z * pa.z), 2.0f);
	const float p_2 = xv_2 / v_2;
	return sqrt(x_2 - p_2);
}

__device__ float colordiffFloat2(const uchar4 &pa, const uchar4 &pb) {
	float3 delta = make_float3((float)pa.x - (float)pb.x, (float)pa.y - (float)pb.y, (float)pa.z - (float)pb.z);
	return length(delta);
}

/*
 * Colour weighting as suggested in:
 * C. Kuster et al. Spatio-Temporal Geometry Fusion for Multiple Hybrid Cameras using Moving Least Squares Surfaces. 2014.
 * c = colour distance
 */
 __device__ float colourWeighting(float c) {
	const float h = c_hashParams.m_colourSmoothing;
	if (c >= h) return 0.0f;
	float ch = c / h;
	ch = 1.0f - ch*ch;
	return ch*ch*ch*ch;
}

#define WINDOW_RADIUS 5

__device__ float mlsCamera(int cam, const float3 &mPos, uchar4 c1, float3 &wpos) {
	const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

	const float3 pf = camera.poseInverse * mPos;
	float3 pos = make_float3(0.0f, 0.0f, 0.0f);
	const uint2 screenPos = make_uint2(camera.params.cameraToKinectScreenInt(pf));
	float weights = 0.0f;

	//#pragma unroll
	for (int v=-WINDOW_RADIUS; v<=WINDOW_RADIUS; ++v) {
		for (int u=-WINDOW_RADIUS; u<=WINDOW_RADIUS; ++u) {
			//if (screenPos.x+u < width && screenPos.y+v < height) {	//on screen
				float depth = tex2D<float>(camera.depth, screenPos.x+u, screenPos.y+v);
				const float3 camPos = camera.params.kinectDepthToSkeleton(screenPos.x+u, screenPos.y+v, depth);
				float weight = ftl::cuda::spatialWeighting(length(pf - camPos), c_hashParams.m_spatialSmoothing);

				if (weight > 0.0f) {
					uchar4 c2 = tex2D<uchar4>(camera.colour, screenPos.x+u, screenPos.y+v);
					weight *= colourWeighting(colordiffFloat2(c1,c2));

					if (weight > 0.0f) {
						wpos += weight* (camera.pose * camPos);
						weights += weight;
					}
				}			
			//}
		}
	}

	//wpos += (camera.pose * pos);

	return weights;
}

__device__ float mlsCameraNoColour(int cam, const float3 &mPos, uchar4 c1, const float4 &norm, float3 &wpos, float h) {
	const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

	const float3 pf = camera.poseInverse * mPos;
	float3 pos = make_float3(0.0f, 0.0f, 0.0f);
	const uint2 screenPos = make_uint2(camera.params.cameraToKinectScreenInt(pf));
	float weights = 0.0f;

	//#pragma unroll
	for (int v=-WINDOW_RADIUS; v<=WINDOW_RADIUS; ++v) {
		for (int u=-WINDOW_RADIUS; u<=WINDOW_RADIUS; ++u) {
			//if (screenPos.x+u < width && screenPos.y+v < height) {	//on creen
				float depth = tex2D<float>(camera.depth, screenPos.x+u, screenPos.y+v);
				const float3 camPos = camera.params.kinectDepthToSkeleton(screenPos.x+u, screenPos.y+v, depth);

				// TODO:(Nick) dot product of normals < 0 means the point
				// should be ignored with a weight of 0 since it is facing the wrong direction
				// May be good to simply weight using the dot product to give
				// a stronger weight to those whose normals are closer

				float weight = ftl::cuda::spatialWeighting(length(pf - camPos), h);

				if (weight > 0.0f) {
					float4 n2 = tex2D<float4>(camera.normal, screenPos.x+u, screenPos.y+v);
					if (dot(make_float3(norm), make_float3(n2)) > 0.0f) {

						uchar4 c2 = tex2D<uchar4>(camera.colour, screenPos.x+u, screenPos.y+v);

						if (colourWeighting(colordiffFloat2(c1,c2)) > 0.0f) {
							pos += weight*camPos; // (camera.pose * camPos);
							weights += weight;
						}
					}
				}			
			//}
		}
	}

	if (weights > 0.0f) wpos += (camera.pose * (pos / weights)) * weights;

	return weights;
}


__global__ void mls_smooth_kernel(ftl::cuda::TextureObject<float4> output, HashParams hashParams, int numcams, int cam) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const int width = output.width();
	const int height = output.height();

	const DepthCameraCUDA &mainCamera = c_cameras[cam];

	if (x < width && y < height) {

		const float depth = tex2D<float>(mainCamera.depth, x, y);
		const uchar4 c1 = tex2D<uchar4>(mainCamera.colour, x, y);
		const float4 norm = tex2D<float4>(mainCamera.normal, x, y);
		//if (x == 400 && y == 200) printf("NORMX: %f\n", norm.x);

		float3 wpos = make_float3(0.0f);
		float3 wnorm = make_float3(0.0f);
		float weights = 0.0f;

		if (depth >= mainCamera.params.m_sensorDepthWorldMin && depth <= mainCamera.params.m_sensorDepthWorldMax) {
			float3 mPos = mainCamera.pose * mainCamera.params.kinectDepthToSkeleton(x, y, depth);

			if ((!(hashParams.m_flags & ftl::voxhash::kFlagClipping)) || (mPos.x > hashParams.m_minBounds.x && mPos.x < hashParams.m_maxBounds.x &&
					mPos.y > hashParams.m_minBounds.y && mPos.y < hashParams.m_maxBounds.y &&
					mPos.z > hashParams.m_minBounds.z && mPos.z < hashParams.m_maxBounds.z)) {

				if (hashParams.m_flags & ftl::voxhash::kFlagMLS) {
					for (uint cam2=0; cam2<numcams; ++cam2) {
						//if (cam2 == cam) weights += mlsCameraNoColour(cam2, mPos, c1, wpos, c_hashParams.m_spatialSmoothing*0.1f); //weights += 0.5*mlsCamera(cam2, mPos, c1, wpos);
						weights += mlsCameraNoColour(cam2, mPos, c1, norm, wpos, c_hashParams.m_spatialSmoothing); //*((cam == cam2)? 0.1f : 5.0f));

						// Previous approach
						//if (cam2 == cam) continue;
						//weights += mlsCameraBest(cam2, mPos, c1, wpos);
					}
					wpos /= weights;
				} else {
					weights = 1000.0f;
					wpos = mPos;
				} 

				//output(x,y) = (weights >= hashParams.m_confidenceThresh) ? make_float4(wpos, 0.0f) : make_float4(MINF,MINF,MINF,MINF);

				if (weights >= hashParams.m_confidenceThresh) output(x,y) = make_float4(wpos, 0.0f);

				//const uint2 screenPos = make_uint2(mainCamera.params.cameraToKinectScreenInt(mainCamera.poseInverse * wpos));
				//if (screenPos.x < output.width() && screenPos.y < output.height()) {
				//	output(screenPos.x,screenPos.y) = (weights >= hashParams.m_confidenceThresh) ? make_float4(wpos, 0.0f) : make_float4(MINF,MINF,MINF,MINF);
				//}
			}
		}
	}
}

void ftl::cuda::mls_smooth(TextureObject<float4> &output, const HashParams &hashParams, int numcams, int cam, cudaStream_t stream) {
	const dim3 gridSize((output.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (output.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	mls_smooth_kernel<<<gridSize, blockSize, 0, stream>>>(output, hashParams, numcams, cam);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}


// ===== Render Depth using MLS ================================================

#define MAX_UPSAMPLE 5
#define SAMPLE_BUFFER ((2*MAX_UPSAMPLE+1)*(2*MAX_UPSAMPLE+1))
#define WARP_SIZE 32
#define BLOCK_WIDTH 4
#define MLS_RADIUS 5
#define MLS_WIDTH (2*MLS_RADIUS+1)
#define MLS_SAMPLES (MLS_WIDTH*MLS_WIDTH)

__global__ void mls_render_depth_kernel(const TextureObject<int> input, TextureObject<int> output, SplatParams params, int numcams) {
	/*const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const int width = output.width();
	const int height = output.height();

	if (x < width && y < height) {

		const float depth = tex2D<float>(mainCamera.depth, x, y);
		const uchar4 c1 = tex2D<uchar4>(mainCamera.colour, x, y);
		const float4 norm = tex2D<float4>(mainCamera.normal, x, y);
		//if (x == 400 && y == 200) printf("NORMX: %f\n", norm.x);

		float3 wpos = make_float3(0.0f);
		float3 wnorm = make_float3(0.0f);
		float weights = 0.0f;

		if (depth >= mainCamera.params.m_sensorDepthWorldMin && depth <= mainCamera.params.m_sensorDepthWorldMax) {
			float3 mPos = mainCamera.pose * mainCamera.params.kinectDepthToSkeleton(x, y, depth);

			if ((!(hashParams.m_flags & ftl::voxhash::kFlagClipping)) || (mPos.x > hashParams.m_minBounds.x && mPos.x < hashParams.m_maxBounds.x &&
					mPos.y > hashParams.m_minBounds.y && mPos.y < hashParams.m_maxBounds.y &&
					mPos.z > hashParams.m_minBounds.z && mPos.z < hashParams.m_maxBounds.z)) {

				if (hashParams.m_flags & ftl::voxhash::kFlagMLS) {
					for (uint cam2=0; cam2<numcams; ++cam2) {
						//if (cam2 == cam) weights += mlsCameraNoColour(cam2, mPos, c1, wpos, c_hashParams.m_spatialSmoothing*0.1f); //weights += 0.5*mlsCamera(cam2, mPos, c1, wpos);
						weights += mlsCameraNoColour(cam2, mPos, c1, norm, wpos, c_hashParams.m_spatialSmoothing); //*((cam == cam2)? 0.1f : 5.0f));

						// Previous approach
						//if (cam2 == cam) continue;
						//weights += mlsCameraBest(cam2, mPos, c1, wpos);
					}
					wpos /= weights;
				} else {
					weights = 1000.0f;
					wpos = mPos;
				} 

				//output(x,y) = (weights >= hashParams.m_confidenceThresh) ? make_float4(wpos, 0.0f) : make_float4(MINF,MINF,MINF,MINF);

				//if (weights >= hashParams.m_confidenceThresh) output(x,y) = make_float4(wpos, 0.0f);

				const uint2 screenPos = make_uint2(mainCamera.params.cameraToKinectScreenInt(mainCamera.poseInverse * wpos));
				if (screenPos.x < output.width() && screenPos.y < output.height()) {
					output(screenPos.x,screenPos.y) = (weights >= hashParams.m_confidenceThresh) ? make_float4(wpos, 0.0f) : make_float4(MINF,MINF,MINF,MINF);
				}
			}
		}
	}*/
}


void ftl::cuda::mls_render_depth(const TextureObject<int> &input, TextureObject<int> &output, const SplatParams &params, int numcams, cudaStream_t stream) {
	const dim3 gridSize((output.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (output.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	mls_render_depth_kernel<<<gridSize, blockSize, 0, stream>>>(input, output, params, numcams);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}
