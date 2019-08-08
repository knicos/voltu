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

__global__ void clear_to_zero_kernel(ftl::cuda::TextureObject<float> depth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		depth(x,y) = 0.0f; //PINF;
	}
}

void ftl::cuda::clear_to_zero(const ftl::cuda::TextureObject<float> &depth, cudaStream_t stream) {
	const dim3 clear_gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 clear_blockSize(T_PER_BLOCK, T_PER_BLOCK);
	clear_to_zero_kernel<<<clear_gridSize, clear_blockSize, 0, stream>>>(depth);
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

__global__ void clear_colour_kernel(ftl::cuda::TextureObject<float4> depth) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < depth.width() && y < depth.height()) {
		depth(x,y) = make_float4(0.0f);
	}
}

void ftl::cuda::clear_colour(const ftl::cuda::TextureObject<float4> &depth, cudaStream_t stream) {
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

__global__ void point_cloud_kernel(ftl::cuda::TextureObject<float4> output, DepthCameraCUDA depthCameraData)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const int width = depthCameraData.params.m_imageWidth;
	const int height = depthCameraData.params.m_imageHeight;

	if (x < width && y < height) {
		float depth = tex2D<float>(depthCameraData.depth, x, y);

		output(x,y) = (depth >= depthCameraData.params.m_sensorDepthWorldMin && depth <= depthCameraData.params.m_sensorDepthWorldMax) ?
			make_float4(depthCameraData.pose * depthCameraData.params.kinectDepthToSkeleton(x, y, depth), 0.0f) :
			make_float4(MINF, MINF, MINF, MINF);
	}
}

void ftl::cuda::point_cloud(ftl::cuda::TextureObject<float4> &output, const DepthCameraCUDA &depthCameraData, cudaStream_t stream) {
	const dim3 gridSize((depthCameraData.params.m_imageWidth + T_PER_BLOCK - 1)/T_PER_BLOCK, (depthCameraData.params.m_imageHeight + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	point_cloud_kernel<<<gridSize, blockSize, 0, stream>>>(output, depthCameraData);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

/// ===== NORMALS =====


__global__ void compute_normals_kernel(const ftl::cuda::TextureObject<float> input, ftl::cuda::TextureObject<float4> output, DepthCameraCUDA camera)
{
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	const int width = output.width();

	if(x >= output.width() || y >= output.height()) return;

	output(x,y) = make_float4(MINF, MINF, MINF, MINF);

	if(x > 0 && x < output.width()-1 && y > 0 && y < output.height()-1)
	{
		const float3 CC = camera.pose * camera.params.kinectDepthToSkeleton(x,y,input(x,y)); //input[(y+0)*width+(x+0)];
		const float3 PC = camera.pose * camera.params.kinectDepthToSkeleton(x,y,input(x,y+1)); //input[(y+1)*width+(x+0)];
		const float3 CP = camera.pose * camera.params.kinectDepthToSkeleton(x,y,input(x+1,y)); //input[(y+0)*width+(x+1)];
		const float3 MC = camera.pose * camera.params.kinectDepthToSkeleton(x,y,input(x,y-1)); //input[(y-1)*width+(x+0)];
		const float3 CM = camera.pose * camera.params.kinectDepthToSkeleton(x,y,input(x-1,y)); //input[(y+0)*width+(x-1)];

		if(CC.x != MINF && PC.x != MINF && CP.x != MINF && MC.x != MINF && CM.x != MINF)
		{
			const float3 n = cross(PC-MC, CP-CM);
			const float  l = length(n);

			if(l > 0.0f)
			{
				//if (x == 400 && y == 200) printf("Cam NORMX: %f\n", (n/-l).x);
				output(x,y) = make_float4(n.x/-l, n.y/-l, n.z/-l, 0.0f); //make_float4(n/-l, 1.0f);
			}
		}
	}
}

void ftl::cuda::compute_normals(const ftl::cuda::TextureObject<float> &input, ftl::cuda::TextureObject<float4> &output, const DepthCameraCUDA &camera, cudaStream_t stream) {
	const dim3 gridSize((output.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (output.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	compute_normals_kernel<<<gridSize, blockSize, 0, stream>>>(input, output, camera);

#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
	//cutilCheckMsg(__FUNCTION__);
#endif
}