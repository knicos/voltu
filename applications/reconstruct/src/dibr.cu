#include "splat_render_cuda.hpp"
#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>

#include "splat_params.hpp"
#include <ftl/depth_camera.hpp>

#define T_PER_BLOCK 8

using ftl::cuda::TextureObject;
using ftl::render::SplatParams;

extern __constant__ ftl::voxhash::DepthCameraCUDA c_cameras[MAX_CAMERAS];

__global__ void clearColourKernel(TextureObject<uchar4> colour) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < colour.width() && y < colour.height()) {
		//depth(x,y) = 0x7f800000; //PINF;
		colour(x,y) = make_uchar4(76,76,82,0);
	}
}

__global__ void dibr_kernel(
    TextureObject<float> depth_in,
    TextureObject<uchar4> colour_out, int numcams, SplatParams params) {

    const int i = threadIdx.y*blockDim.y + threadIdx.x;
    const int bx = blockIdx.x*blockDim.x;
    const int by = blockIdx.y*blockDim.y;
    const int x = bx + threadIdx.x;
    const int y = by + threadIdx.y;
    
    for (int j=0; j<numcams; ++j) {
        const ftl::voxhash::DepthCameraCUDA camera = c_cameras[j];
	
		float d = tex2D<float>(camera.depth, x, y);
		if (d < 0.01f)	continue;
        if (d >= params.camera.m_sensorDepthWorldMax) continue;
        
        const float3 worldPos = camera.pose * camera.params.kinectDepthToSkeleton(x, y, d);
        
        const float3 camPos = params.m_viewMatrix * worldPos;
	    const float2 screenPosf = params.camera.cameraToKinectScreenFloat(camPos);
	    const uint2 screenPos = make_uint2(make_int2(screenPosf));

        if (camPos.z < params.camera.m_sensorDepthWorldMin) continue;

        const unsigned int cx = screenPos.x;
        const unsigned int cy = screenPos.y;


        if (cx < colour_out.width() && cy < colour_out.height()) {
            float camd = depth_in.tex2D((int)cx,(int)cy);
            //atomicMin(&depth(x,y), idepth);
            float camdiff = fabs(camPos.z-camd);
            if (camdiff < 0.1f) {
           		colour_out(cx,cy) = tex2D<uchar4>(camera.colour,x,y);
            } else {
				//colour_out(cx,cy) = make_uchar4(camdiff * 100, 0, 0, 255);
			}
        }
    }
}

__global__ void dibr_kernel_rev(
    TextureObject<float> depth_in,
    TextureObject<uchar4> colour_out, int numcams, SplatParams params) {

    const int i = threadIdx.y*blockDim.y + threadIdx.x;
    const int bx = blockIdx.x*blockDim.x;
    const int by = blockIdx.y*blockDim.y;
    const int x = bx + threadIdx.x;
	const int y = by + threadIdx.y;
	
	float camd = depth_in.tex2D((int)x,(int)y);
	if (camd < 0.01f)	return;
	if (camd >= params.camera.m_sensorDepthWorldMax) return;
	
	const float3 worldPos = params.m_viewMatrixInverse * params.camera.kinectDepthToSkeleton(x, y, camd);
    
    for (int j=0; j<numcams; ++j) {
		const ftl::voxhash::DepthCameraCUDA camera = c_cameras[j];
		
		const float3 camPos = camera.poseInverse * worldPos;
	    const float2 screenPosf = camera.params.cameraToKinectScreenFloat(camPos);
		const uint2 screenPos = make_uint2(make_int2(screenPosf));
		
		if (camPos.z < params.camera.m_sensorDepthWorldMin) continue;

		const unsigned int cx = screenPos.x;
        const unsigned int cy = screenPos.y;

        if (cx < camera.params.m_imageWidth && cy < camera.params.m_imageHeight) {
			float d = tex2D<float>(camera.depth, (int)cx, (int)cy);
			float camdiff = fabs(camPos.z-d);
            if (camdiff < 0.1f) {
            	colour_out(x,y) = tex2D<uchar4>(camera.colour,cx,cy);
            } else {
				//colour_out(x,y) = make_uchar4(camdiff * 100, 0, 0, 255);
			}
		}
    }
}

void ftl::cuda::dibr(const TextureObject<float> &depth_in,
    const TextureObject<uchar4> &colour_out, int numcams, const SplatParams &params, cudaStream_t stream) {

    const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    clearColourKernel<<<gridSize, blockSize, 0, stream>>>(colour_out);

    dibr_kernel_rev<<<gridSize, blockSize, 0, stream>>>(depth_in, colour_out, numcams, params);
    cudaSafeCall( cudaGetLastError() );
}
