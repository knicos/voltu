#include "splat_params.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

#include <ftl/cuda/weighting.hpp>

#define T_PER_BLOCK 8
#define UPSAMPLE_FACTOR 1.8f
#define WARP_SIZE 32
#define DEPTH_THRESHOLD 0.05f
#define UPSAMPLE_MAX 60
#define MAX_ITERATIONS 32  // Note: Must be multiple of 32
#define SPATIAL_SMOOTHING 0.005f

using ftl::cuda::TextureObject;
using ftl::render::SplatParams;

/*
 * Pass 1: Directly render each camera into virtual view but with no upsampling
 * for sparse points.
 */
 __global__ void dibr_merge_kernel(TextureObject<float4> points, TextureObject<int> depth, int cam, SplatParams params) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float3 worldPos = make_float3(points.tex2D(x, y));
	if (worldPos.x == MINF) return;

    // Find the virtual screen position of current point
	const float3 camPos = params.m_viewMatrix * worldPos;
	if (camPos.z < params.camera.minDepth) return;
	if (camPos.z > params.camera.maxDepth) return;

	const float d = camPos.z;

	const uint2 screenPos = params.camera.camToScreen<uint2>(camPos);
	const unsigned int cx = screenPos.x;
	const unsigned int cy = screenPos.y;
	if (d > params.camera.minDepth && d < params.camera.maxDepth && cx < depth.width() && cy < depth.height()) {
		// Transform estimated point to virtual cam space and output z
		atomicMin(&depth(cx,cy), d * 1000.0f);
	}
}

__device__ inline float4 make_float4(const uchar4 &c) {
    return make_float4(c.x,c.y,c.z,c.w);
}


#define ENERGY_THRESHOLD 0.1f
#define SMOOTHING_MULTIPLIER_A 10.0f	// For surface search
#define SMOOTHING_MULTIPLIER_B 4.0f		// For z contribution
#define SMOOTHING_MULTIPLIER_C 4.0f		// For colour contribution

/*
 * Pass 2: Accumulate attribute contributions if the points pass a visibility test.
 */
__global__ void dibr_attribute_contrib_kernel(
        TextureObject<uchar4> colour_in,    // Original colour image
        TextureObject<float4> points,       // Original 3D points
        TextureObject<int> depth_in,        // Virtual depth map
        TextureObject<float4> colour_out,   // Accumulated output
        //TextureObject<float4> normal_out,
        TextureObject<float> contrib_out,
        SplatParams params) {
        
	//const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

	const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
	//const int warp = tid / WARP_SIZE;
	const int x = (blockIdx.x*blockDim.x + threadIdx.x) / WARP_SIZE;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float3 worldPos = make_float3(points.tex2D(x, y));
	//const float3 normal = make_float3(tex2D<float4>(camera.normal, x, y));
	if (worldPos.x == MINF) return;
    //const float r = (camera.poseInverse * worldPos).z / camera.params.fx;

	const float3 camPos = params.m_viewMatrix * worldPos;
	if (camPos.z < params.camera.minDepth) return;
	if (camPos.z > params.camera.maxDepth) return;
	const uint2 screenPos = params.camera.camToScreen<uint2>(camPos);

    const int upsample = 8; //min(UPSAMPLE_MAX, int((5.0f*r) * params.camera.fx / camPos.z));

	// Not on screen so stop now...
	if (screenPos.x >= depth_in.width() || screenPos.y >= depth_in.height()) return;
            
    // Is this point near the actual surface and therefore a contributor?
    const float d = ((float)depth_in.tex2D((int)screenPos.x, (int)screenPos.y)/1000.0f);
    //if (abs(d - camPos.z) > DEPTH_THRESHOLD) return;

    // TODO:(Nick) Should just one thread load these to shared mem?
    const float4 colour = make_float4(colour_in.tex2D(x, y));
    //const float4 normal = tex2D<float4>(camera.normal, x, y);

	// Each thread in warp takes an upsample point and updates corresponding depth buffer.
	const int lane = tid % WARP_SIZE;
	for (int i=lane; i<upsample*upsample; i+=WARP_SIZE) {
		const float u = (i % upsample) - (upsample / 2);
		const float v = (i / upsample) - (upsample / 2);

        // Use the depth buffer to determine this pixels 3D position in camera space
        const float d = ((float)depth_in.tex2D(screenPos.x+u, screenPos.y+v)/1000.0f);
		const float3 nearest = params.camera.screenToCam((int)(screenPos.x+u),(int)(screenPos.y+v),d);

        // What is contribution of our current point at this pixel?
        const float weight = ftl::cuda::spatialWeighting(length(nearest - camPos), SMOOTHING_MULTIPLIER_C*(nearest.z/params.camera.fx));
        if (screenPos.x+u < colour_out.width() && screenPos.y+v < colour_out.height() && weight > 0.0f) {  // TODO: Use confidence threshold here
            const float4 wcolour = colour * weight;
			//const float4 wnormal = normal * weight;
			
			//printf("Z %f\n", d);

            // Add this points contribution to the pixel buffer
            atomicAdd((float*)&colour_out(screenPos.x+u, screenPos.y+v), wcolour.x);
            atomicAdd((float*)&colour_out(screenPos.x+u, screenPos.y+v)+1, wcolour.y);
            atomicAdd((float*)&colour_out(screenPos.x+u, screenPos.y+v)+2, wcolour.z);
            atomicAdd((float*)&colour_out(screenPos.x+u, screenPos.y+v)+3, wcolour.w);
            //atomicAdd((float*)&normal_out(screenPos.x+u, screenPos.y+v), wnormal.x);
            //atomicAdd((float*)&normal_out(screenPos.x+u, screenPos.y+v)+1, wnormal.y);
            //atomicAdd((float*)&normal_out(screenPos.x+u, screenPos.y+v)+2, wnormal.z);
            //atomicAdd((float*)&normal_out(screenPos.x+u, screenPos.y+v)+3, wnormal.w);
            atomicAdd(&contrib_out(screenPos.x+u, screenPos.y+v), weight);
        }
	}
}




__global__ void dibr_normalise_kernel(
    TextureObject<float4> colour_in,
    TextureObject<uchar4> colour_out,
    //TextureObject<float4> normals,
    TextureObject<float> contribs) {
const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

if (x < colour_in.width() && y < colour_in.height()) {
    const float4 colour = colour_in.tex2D((int)x,(int)y);
    //const float4 normal = normals.tex2D((int)x,(int)y);
    const float contrib = contribs.tex2D((int)x,(int)y);

    if (contrib > 0.0f) {
        colour_out(x,y) = make_uchar4(colour.x / contrib, colour.y / contrib, colour.z / contrib, 0);
        //normals(x,y) = normal / contrib;
    }
}
}
