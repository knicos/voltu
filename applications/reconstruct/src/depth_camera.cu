#include <ftl/cuda_common.hpp>
#include <ftl/cuda_util.hpp>
#include <ftl/depth_camera.hpp>
#include "depth_camera_cuda.hpp"

#define T_PER_BLOCK 16
#define MINF __int_as_float(0xff800000)

using ftl::voxhash::DepthCameraCUDA;
using ftl::voxhash::HashData;
using ftl::voxhash::HashParams;
using ftl::cuda::TextureObject;
using ftl::render::SplatParams;

extern __constant__ ftl::voxhash::DepthCameraCUDA c_cameras[MAX_CAMERAS];
extern __constant__ HashParams c_hashParams;

__global__ void clear_depth_kernel(ftl::cuda::TextureObject<float> depth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		depth(x,y) = 1000.0f; //PINF;
		//colour(x,y) = make_uchar4(76,76,82,0);
	}
}

void ftl::cuda::clear_depth(const ftl::cuda::TextureObject<float> &depth, cudaStream_t stream) {
	const dim3 clear_gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 clear_blockSize(T_PER_BLOCK, T_PER_BLOCK);
	clear_depth_kernel<<<clear_gridSize, clear_blockSize, 0, stream>>>(depth);
}

__global__ void clear_depth_kernel(ftl::cuda::TextureObject<int> depth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		depth(x,y) = 0x7FFFFFFF; //PINF;
		//colour(x,y) = make_uchar4(76,76,82,0);
	}
}

void ftl::cuda::clear_depth(const ftl::cuda::TextureObject<int> &depth, cudaStream_t stream) {
	const dim3 clear_gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 clear_blockSize(T_PER_BLOCK, T_PER_BLOCK);
	clear_depth_kernel<<<clear_gridSize, clear_blockSize, 0, stream>>>(depth);
}

__global__ void clear_points_kernel(ftl::cuda::TextureObject<float4> depth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		depth(x,y) = make_float4(MINF,MINF,MINF,MINF);
		//colour(x,y) = make_uchar4(76,76,82,0);
	}
}

void ftl::cuda::clear_points(const ftl::cuda::TextureObject<float4> &depth, cudaStream_t stream) {
	const dim3 clear_gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 clear_blockSize(T_PER_BLOCK, T_PER_BLOCK);
	clear_points_kernel<<<clear_gridSize, clear_blockSize, 0, stream>>>(depth);
}

__global__ void clear_colour_kernel(ftl::cuda::TextureObject<uchar4> depth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		depth(x,y) = make_uchar4(76,76,76,76);
		//colour(x,y) = make_uchar4(76,76,82,0);
	}
}

void ftl::cuda::clear_colour(const ftl::cuda::TextureObject<uchar4> &depth, cudaStream_t stream) {
	const dim3 clear_gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 clear_blockSize(T_PER_BLOCK, T_PER_BLOCK);
	clear_colour_kernel<<<clear_gridSize, clear_blockSize, 0, stream>>>(depth);
}

// ===== Type convert =====

template <typename A, typename B>
__global__ void convert_kernel(const ftl::cuda::TextureObject<A> in, ftl::cuda::TextureObject<B> out, float scale) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < in.width() && y < in.height()) {
		out(x,y) = ((float)in.tex2D((int)x,(int)y)) * scale;
	}
}

void ftl::cuda::float_to_int(const ftl::cuda::TextureObject<float> &in, ftl::cuda::TextureObject<int> &out, float scale, cudaStream_t stream) {
	const dim3 gridSize((in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);
	convert_kernel<float,int><<<gridSize, blockSize, 0, stream>>>(in, out, scale);
}

void ftl::cuda::int_to_float(const ftl::cuda::TextureObject<int> &in, ftl::cuda::TextureObject<float> &out, float scale, cudaStream_t stream) {
	const dim3 gridSize((in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);
	convert_kernel<int,float><<<gridSize, blockSize, 0, stream>>>(in, out, scale);
}

/// ===== MLS Smooth

// TODO:(Nick) Put this in a common location (used in integrators.cu)
extern __device__ float spatialWeighting(float r);
extern __device__ float spatialWeighting(float r, float h);

/*
 * Kim, K., Chalidabhongse, T. H., Harwood, D., & Davis, L. (2005).
 * Real-time foreground-background segmentation using codebook model.
 * Real-Time Imaging. https://doi.org/10.1016/j.rti.2004.12.004
 */
 __device__ float colordiffFloat(const uchar4 &pa, const uchar4 &pb) {
	const float x_2 = pb.x * pb.x + pb.y * pb.y + pb.z * pb.z;
	const float v_2 = pa.x * pa.x + pa.y * pa.y + pa.z * pa.z;
	const float xv_2 = pow(pb.x * pa.x + pb.y * pa.y + pb.z * pa.z, 2);
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
				float weight = spatialWeighting(length(pf - camPos));

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

__device__ float mlsCameraNoColour(int cam, const float3 &mPos, uchar4 c1, float3 &wpos, float h) {
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

				float weight = spatialWeighting(length(pf - camPos), h);

				if (weight > 0.0f) {
					uchar4 c2 = tex2D<uchar4>(camera.colour, screenPos.x+u, screenPos.y+v);

					if (colourWeighting(colordiffFloat2(c1,c2)) > 0.0f) {
						pos += weight*camPos; // (camera.pose * camPos);
						weights += weight;
					}
				}			
			//}
		}
	}

	if (weights > 0.0f) wpos += (camera.pose * (pos / weights)) * weights;

	return weights;
}

__device__ float mlsCameraBest(int cam, const float3 &mPos, uchar4 c1, float3 &wpos) {
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
				float weight = spatialWeighting(length(pf - camPos));

				if (weight > 0.0f) {
					uchar4 c2 = tex2D<uchar4>(camera.colour, screenPos.x+u, screenPos.y+v);
					weight *= colourWeighting(colordiffFloat2(c1,c2));

					if (weight > weights) {
						pos = weight* (camera.pose * camPos);
						weights = weight;
					}
				}			
			//}
		}
	}

	wpos += pos;
	//wpos += (camera.pose * pos);

	return weights;
}

__device__ float mlsCameraPoint(int cam, const float3 &mPos, uchar4 c1, float3 &wpos) {
	const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

	const float3 pf = camera.poseInverse * mPos;
	float3 pos = make_float3(0.0f, 0.0f, 0.0f);
	const uint2 screenPos = make_uint2(camera.params.cameraToKinectScreenInt(pf));
	float weights = 0.0f;


	//float depth = tex2D<float>(camera.depth, screenPos.x, screenPos.y);
	const float3 worldPos = make_float3(tex2D<float4>(camera.points, screenPos.x, screenPos.y));
	if (worldPos.z == MINF) return 0.0f;

	float weight = spatialWeighting(length(mPos - worldPos));

	if (weight > 0.0f) {
		wpos += weight* (worldPos);
		weights += weight;
	}

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
						weights += mlsCameraNoColour(cam2, mPos, c1, wpos, c_hashParams.m_spatialSmoothing); //*((cam == cam2)? 0.1f : 5.0f));

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

#define RESAMPLE_RADIUS 7

__global__ void mls_resample_kernel(ftl::cuda::TextureObject<int> depthin, ftl::cuda::TextureObject<uchar4> colourin, ftl::cuda::TextureObject<float> depthout, HashParams hashParams, int numcams, SplatParams params) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const int width = depthin.width();
	const int height = depthin.height();

	if (x < width && y < height) {

		//const int depth = depthin.tex2D((int)x, (int)y);
		//if (depth != 0x7FFFFFFF) {
		//	depthout(x,y) = (float)depth / 1000.0f;
		//	return;
		//}

		struct map_t {
			int d;
			int quad;
		};

		map_t mappings[5];
		int mapidx = 0;

		for (int v=-RESAMPLE_RADIUS; v<=RESAMPLE_RADIUS; ++v) {
			for (int u=-RESAMPLE_RADIUS; u<=RESAMPLE_RADIUS; ++u) {

				const int depth = depthin.tex2D((int)x+u, (int)y+v);
				const uchar4 c1 = colourin.tex2D((int)x+u, (int)y+v);
				
				if (depth != 0x7FFFFFFF) {
					int i=0;
					for (i=0; i<mapidx; ++i) {
						if (abs(mappings[i].d - depth) < 100) {
							if (u < 0 && v < 0) mappings[i].quad |= 0x1;
							if (u > 0 && v < 0) mappings[i].quad |= 0x2;
							if (u > 0 && v > 0) mappings[i].quad |= 0x4;
							if (u < 0 && v > 0) mappings[i].quad |= 0x8;
							break;
						}
					}
					if (i == mapidx && i < 5) {
						mappings[mapidx].d = depth;
						mappings[mapidx].quad = 0;
						if (u < 0 && v < 0) mappings[mapidx].quad |= 0x1;
						if (u > 0 && v < 0) mappings[mapidx].quad |= 0x2;
						if (u > 0 && v > 0) mappings[mapidx].quad |= 0x4;
						if (u < 0 && v > 0) mappings[mapidx].quad |= 0x8;
						++mapidx;
					} else {
						//printf("EXCEEDED\n");
					}
				}
			}
		}

		int bestdepth = 1000000;
		//int count = 0;
		for (int i=0; i<mapidx; ++i) {
			if (__popc(mappings[i].quad) >= 3 && mappings[i].d < bestdepth) bestdepth = mappings[i].d;
			//if (mappings[i].quad == 15 && mappings[i].d < bestdepth) bestdepth = mappings[i].d;
			//if (mappings[i].quad == 15) count ++;
		}

		//depthout(x,y) = (mapidx == 5) ? 3.0f : 0.0f;

		if (bestdepth < 1000000) {
			depthout(x,y) = (float)bestdepth / 1000.0f;
		}
	}
}

void ftl::cuda::mls_resample(const TextureObject<int> &depthin, const TextureObject<uchar4> &colourin, TextureObject<float> &depthout, const HashParams &hashParams, int numcams, const SplatParams &params, cudaStream_t stream) {
	const dim3 gridSize((depthin.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depthin.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	mls_resample_kernel<<<gridSize, blockSize, 0, stream>>>(depthin, colourin, depthout, hashParams, numcams, params);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}


/// ===== Median Filter ======

#define WINDOW_SIZE 3
#define MEDIAN_RADIUS 3
#define MEDIAN_SIZE (((MEDIAN_RADIUS*2)+1)*((MEDIAN_RADIUS*2)+1))

__global__ void medianFilterKernel(TextureObject<int> inputImageKernel, TextureObject<float> outputImagekernel)
{
	// Set row and colum for thread.
	int row = blockIdx.y * blockDim.y + threadIdx.y;
	int col = blockIdx.x * blockDim.x + threadIdx.x;
	int filterVector[MEDIAN_SIZE] = {0};   //Take fiter window
	if((row<=MEDIAN_RADIUS) || (col<=MEDIAN_RADIUS) || (row>=inputImageKernel.height()-MEDIAN_RADIUS) || (col>=inputImageKernel.width()-MEDIAN_RADIUS))
				outputImagekernel(col, row) = 0.0f; //Deal with boundry conditions
	else {
		for (int v = -MEDIAN_RADIUS; v <= MEDIAN_RADIUS; v++) { 
			for (int u = -MEDIAN_RADIUS; u <= MEDIAN_RADIUS; u++){
				filterVector[(v+MEDIAN_RADIUS)*(2*MEDIAN_RADIUS+1)+u+MEDIAN_RADIUS] = inputImageKernel((col+u), (row+v));   // setup the filterign window.
			}
		}
		for (int i = 0; i < MEDIAN_SIZE; i++) {
			for (int j = i + 1; j < MEDIAN_SIZE; j++) {
				if (filterVector[i] > filterVector[j]) { 
					//Swap the variables.
					char tmp = filterVector[i];
					filterVector[i] = filterVector[j];
					filterVector[j] = tmp;
				}
			}
		}
		outputImagekernel(col, row) = (float)filterVector[MEDIAN_SIZE/2+1] / 1000.0f;   //Set the output variables.
	}
}

void ftl::cuda::median_filter(const ftl::cuda::TextureObject<int> &in, ftl::cuda::TextureObject<float> &out, cudaStream_t stream) {
	const dim3 gridSize((in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);
	medianFilterKernel<<<gridSize, blockSize, 0, stream>>>(in, out);
}


/// ===== Hole Fill =====

__device__ inline float distance2(float3 a, float3 b) {
	const float x = a.x-b.x;
	const float y = a.y-b.y;
	const float z = a.z-b.z;
	return x*x+y*y+z*z;
}

#define SPLAT_RADIUS 7
#define SPLAT_BOUNDS (2*SPLAT_RADIUS+T_PER_BLOCK+1)
#define SPLAT_BUFFER_SIZE (SPLAT_BOUNDS*SPLAT_BOUNDS)

__global__ void hole_fill_kernel(
	TextureObject<int> depth_in,
	TextureObject<float> depth_out, DepthCameraParams params) {
	// Read an NxN region and
	// - interpolate a depth value for this pixel
	// - interpolate an rgb value for this pixel
	// Must respect depth discontinuities.
	// How much influence a given neighbour has depends on its depth value

	__shared__ float3 positions[SPLAT_BUFFER_SIZE];

	const float voxelSize = c_hashParams.m_virtualVoxelSize;

	const int i = threadIdx.y*blockDim.y + threadIdx.x;
	const int bx = blockIdx.x*blockDim.x;
	const int by = blockIdx.y*blockDim.y;
	const int x = bx + threadIdx.x;
	const int y = by + threadIdx.y;

	// const float camMinDepth = params.camera.m_sensorDepthWorldMin;
	// const float camMaxDepth = params.camera.m_sensorDepthWorldMax;

	for (int j=i; j<SPLAT_BUFFER_SIZE; j+=T_PER_BLOCK) {
		const unsigned int sx = (j % SPLAT_BOUNDS)+bx-SPLAT_RADIUS;
		const unsigned int sy = (j / SPLAT_BOUNDS)+by-SPLAT_RADIUS;
		if (sx >= depth_in.width() || sy >= depth_in.height()) {
			positions[j] = make_float3(1000.0f,1000.0f, 1000.0f);
		} else {
			positions[j] = params.kinectDepthToSkeleton(sx, sy, (float)depth_in.tex2D((int)sx,(int)sy) / 1000.0f);
		}
	}

	__syncthreads();

	if (x >= depth_in.width() || y >= depth_in.height()) return;

	const float voxelSquared = voxelSize*voxelSize;
	float mindepth = 1000.0f;
	//int minidx = -1;
	// float3 minpos;

	//float3 validPos[MAX_VALID];
	//int validIndices[MAX_VALID];
	//int validix = 0;

	for (int v=-SPLAT_RADIUS; v<=SPLAT_RADIUS; ++v) {
		for (int u=-SPLAT_RADIUS; u<=SPLAT_RADIUS; ++u) {
			//const int idx = (threadIdx.y+v)*SPLAT_BOUNDS+threadIdx.x+u;
			const int idx = (threadIdx.y+v+SPLAT_RADIUS)*SPLAT_BOUNDS+threadIdx.x+u+SPLAT_RADIUS;

			float3 posp = positions[idx];
			const float d = posp.z;
			//if (d < camMinDepth || d > camMaxDepth) continue;

			float3 pos = params.kinectDepthToSkeleton(x, y, d);
			float dist = distance2(pos, posp);

			if (dist < voxelSquared) {
				// Valid so check for minimum
				//validPos[validix] = pos;
				//validIndices[validix++] = idx;
				if (d < mindepth) {
					mindepth = d;
					//minidx = idx;
					// minpos = pos;
				}
			}
		}
	}

	depth_out(x,y) = mindepth;
}

void ftl::cuda::hole_fill(const TextureObject<int> &depth_in,
	const TextureObject<float> &depth_out, const DepthCameraParams &params, cudaStream_t stream) 
{

	const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	hole_fill_kernel<<<gridSize, blockSize, 0, stream>>>(depth_in, depth_out, params);
	cudaSafeCall( cudaGetLastError() );

	#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	#endif
}


/// ===== Point cloud from depth =====

__global__ void point_cloud_kernel(float3* output, DepthCameraCUDA depthCameraData)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const int width = depthCameraData.params.m_imageWidth;
	const int height = depthCameraData.params.m_imageHeight;

	if (x < width && y < height) {
		float depth = tex2D<float>(depthCameraData.depth, x, y);

		output[y*width+x] = (depth >= depthCameraData.params.m_sensorDepthWorldMin && depth <= depthCameraData.params.m_sensorDepthWorldMax) ?
			depthCameraData.params.kinectDepthToSkeleton(x, y, depth) :
			make_float3(MINF, MINF, MINF);
	}
}

void ftl::cuda::point_cloud(float3* output, const DepthCameraCUDA &depthCameraData, cudaStream_t stream) {
	const dim3 gridSize((depthCameraData.params.m_imageWidth + T_PER_BLOCK - 1)/T_PER_BLOCK, (depthCameraData.params.m_imageHeight + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	point_cloud_kernel<<<gridSize, blockSize, 0, stream>>>(output, depthCameraData);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

/// ===== NORMALS =====


__global__ void compute_normals_kernel(const float3 *input, ftl::cuda::TextureObject<float4> output)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const int width = output.width();

	if(x >= output.width() || y >= output.height()) return;

	output(x,y) = make_float4(MINF, MINF, MINF, MINF);

	if(x > 0 && x < output.width()-1 && y > 0 && y < output.height()-1)
	{
		// TODO:(Nick) Should use a 7x7 window
		const float3 CC = input[(y+0)*width+(x+0)];
		const float3 PC = input[(y+1)*width+(x+0)];
		const float3 CP = input[(y+0)*width+(x+1)];
		const float3 MC = input[(y-1)*width+(x+0)];
		const float3 CM = input[(y+0)*width+(x-1)];

		if(CC.x != MINF && PC.x != MINF && CP.x != MINF && MC.x != MINF && CM.x != MINF)
		{
			const float3 n = cross(PC-MC, CP-CM);
			//const float  l = length(n);

			//if(l > 0.0f)
			//{
				output(x,y) = make_float4(n, 1.0f); //make_float4(n/-l, 1.0f);
			//}
		}
	}
}

void ftl::cuda::compute_normals(const float3 *input, ftl::cuda::TextureObject<float4> *output, cudaStream_t stream) {
	const dim3 gridSize((output->width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (output->height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	compute_normals_kernel<<<gridSize, blockSize, 0, stream>>>(input, *output);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}