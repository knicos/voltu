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


__global__ void dibr_depthmap_kernel(
    TextureObject<int> depth, int numcams, SplatParams params) {

    const int i = threadIdx.y*blockDim.y + threadIdx.x;
    const int bx = blockIdx.x*blockDim.x;
    const int by = blockIdx.y*blockDim.y;
    const int x = bx + threadIdx.x;
    const int y = by + threadIdx.y;
    
    for (int j=0; j<numcams; ++j) {
        const ftl::voxhash::DepthCameraCUDA camera = c_cameras[j];
	
		float4 d = tex2D<float4>(camera.points, x, y);
		if (d.z < 0.0f) continue;
        //if (d >= params.camera.m_sensorDepthWorldMax) continue;
        
        //const float3 worldPos = camera.pose * camera.params.kinectDepthToSkeleton(x, y, d);
        
        const float3 worldPos = make_float3(d);
        const float3 camPos = params.m_viewMatrix * worldPos;
	    const float2 screenPosf = params.camera.cameraToKinectScreenFloat(camPos);
	    const uint2 screenPos = make_uint2(make_int2(screenPosf));

        if (camPos.z < params.camera.m_sensorDepthWorldMin) continue;

        const unsigned int cx = screenPos.x;
        const unsigned int cy = screenPos.y;


        if (cx < depth.width() && cy < depth.height()) {
            //float camd = depth_in.tex2D((int)cx,(int)cy);
            //atomicMin(&depth(x,y), idepth);
            //float camdiff = fabs(camPos.z-camd);
            //if (camdiff < 0.1f) {
           		//colour_out(cx,cy) = tex2D<uchar4>(camera.colour,x,y);
            //} else {
				//colour_out(cx,cy) = make_uchar4(camdiff * 100, 0, 0, 255);
            //}
            
            atomicMin(&depth(cx,cy), camPos.z * 1000.0f);
        }
    }
}


__global__ void dibr_kernel(
    TextureObject<int> depth_in,
    TextureObject<uchar4> colour_out, int numcams, SplatParams params) {

    const int i = threadIdx.y*blockDim.y + threadIdx.x;
    const int bx = blockIdx.x*blockDim.x;
    const int by = blockIdx.y*blockDim.y;
    const int x = bx + threadIdx.x;
    const int y = by + threadIdx.y;
    
    for (int j=0; j<numcams; ++j) {
        const ftl::voxhash::DepthCameraCUDA camera = c_cameras[j];
	
		float4 d = tex2D<float4>(camera.points, x, y);
		if (d.z < 0.0f) continue;
        //if (d >= params.camera.m_sensorDepthWorldMax) continue;
        
        //const float3 worldPos = camera.pose * camera.params.kinectDepthToSkeleton(x, y, d);
        
        const float3 worldPos = make_float3(d);
        const float3 camPos = params.m_viewMatrix * worldPos;
	    const float2 screenPosf = params.camera.cameraToKinectScreenFloat(camPos);
	    const uint2 screenPos = make_uint2(make_int2(screenPosf));

        if (camPos.z < params.camera.m_sensorDepthWorldMin) continue;

        const unsigned int cx = screenPos.x;
        const unsigned int cy = screenPos.y;


        if (cx < colour_out.width() && cy < colour_out.height()) {
            //float camd = depth_in.tex2D((int)cx,(int)cy);
            //atomicMin(&depth(x,y), idepth);
            //float camdiff = fabs(camPos.z-camd);
            //if (camdiff < 0.1f) {

            if (depth_in(cx,cy) == (int)(camPos.z * 1000.0f)) {
				   colour_out(cx,cy) = tex2D<uchar4>(camera.colour,x,y);
				   //colour_out(cx,cy) = (j==0) ? make_uchar4(20,255,0,255) : make_uchar4(20,0,255,255);
            }
                   

            //} else {
				//colour_out(cx,cy) = make_uchar4(camdiff * 100, 0, 0, 255);
			//}
        }
    }
}

__device__ inline float4 make_float4(const uchar4 &c) {
    return make_float4(c.x,c.y,c.z,c.w);
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
    float mindiff = 1000.0f;
    float4 col = make_float4(0.0f,0.0f,0.0f,0.0f);
    int count = 0;

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
            
            if (camdiff < mindiff) {
                mindiff = camdiff;
                col += make_float4(tex2D<uchar4>(camera.colour,cx,cy));
                ++count;
            }

            //if (camdiff < 0.1f) {
            //	colour_out(x,y) = tex2D<uchar4>(camera.colour,cx,cy);
            //} else {
				//colour_out(x,y) = make_uchar4(camdiff * 100, 0, 0, 255);
			//}
		}
    }

    if (count > 0) {
        col = col / (float)count;
        colour_out(x,y) = make_uchar4(col.x,col.y,col.z,255);
    } else {
        colour_out(x,y) = make_uchar4(76,76,76,255);
    }
}

void ftl::cuda::dibr(const TextureObject<int> &depth_out,
    const TextureObject<uchar4> &colour_out, int numcams, const SplatParams &params, cudaStream_t stream) {

    const dim3 gridSize((depth_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	clearColourKernel<<<gridSize, blockSize, 0, stream>>>(colour_out);
	
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif

    dibr_depthmap_kernel<<<gridSize, blockSize, 0, stream>>>(depth_out, numcams, params);
    dibr_kernel<<<gridSize, blockSize, 0, stream>>>(depth_out, colour_out, numcams, params);
	cudaSafeCall( cudaGetLastError() );
	
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

void ftl::cuda::dibr(const TextureObject<float> &depth_out,
    const TextureObject<uchar4> &colour_out, int numcams, const SplatParams &params, cudaStream_t stream) {

    const dim3 gridSize((depth_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	clearColourKernel<<<gridSize, blockSize, 0, stream>>>(colour_out);
	
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif

    dibr_kernel_rev<<<gridSize, blockSize, 0, stream>>>(depth_out, colour_out, numcams, params);
	cudaSafeCall( cudaGetLastError() );
	
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}
