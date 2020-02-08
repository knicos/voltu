#include "splat_render_cuda.hpp"
#include "depth_camera_cuda.hpp"
//#include <cuda_runtime.h>

#include <ftl/cuda_matrix_util.hpp>

#include "splat_params.hpp"
#include "mls_cuda.hpp"
#include <ftl/depth_camera.hpp>

#define T_PER_BLOCK 8
#define UPSAMPLE_FACTOR 1.8f
#define WARP_SIZE 32
#define DEPTH_THRESHOLD 0.05f
#define UPSAMPLE_MAX 60
#define MAX_ITERATIONS 32  // Note: Must be multiple of 32
#define SPATIAL_SMOOTHING 0.005f

using ftl::cuda::TextureObject;
using ftl::render::Parameters;

extern __constant__ ftl::voxhash::DepthCameraCUDA c_cameras[MAX_CAMERAS];

__global__ void clearColourKernel(TextureObject<uchar4> colour) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < colour.width() && y < colour.height()) {
		//depth(x,y) = 0x7f800000; //PINF;
		colour(x,y) = make_uchar4(76,76,82,0);
	}
}

__device__ inline bool isStable(const float3 &previous, const float3 &estimate, const SplatParams &params, float d) {
    const float psize = 2.0f * d / params.camera.fx;
    //printf("PSIZE %f\n", psize);
    return fabs(previous.x - estimate.x) <= psize &&
        fabs(previous.y - estimate.y) <= psize &&
        fabs(previous.z - estimate.z) <= psize;
}

// ===== PASS 1 : Gather & Upsample (Depth) ====================================

/*
 * Pass 1: Directly render raw points from all cameras, but upsample the points
 * if their spacing is within smoothing threshold but greater than their pixel
 * size in the original image.
 */
 __global__ void dibr_merge_upsample_kernel(TextureObject<int> depth, int cam, SplatParams params) {
	const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

	const int x = (blockIdx.x*blockDim.x + threadIdx.x) / WARP_SIZE;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float3 worldPos = make_float3(tex2D<float4>(camera.points, x, y));
	//const float3 normal = make_float3(tex2D<float4>(camera.normal, x, y));
	if (worldPos.x == MINF) return;
    const float r = (camera.poseInverse * worldPos).z / camera.params.fx;

	// Get virtual camera ray for splat centre and backface cull if possible
	//const float3 rayOrigin = params.m_viewMatrixInverse * make_float3(0.0f,0.0f,0.0f);
	//const float3 rayDir = normalize(params.m_viewMatrixInverse * params.camera.kinectDepthToSkeleton(x,y,1.0f) - rayOrigin);
	//if (dot(rayDir, normal) > 0.0f) return;

    // Find the virtual screen position of current point
	const float3 camPos = params.m_viewMatrix * worldPos;
	if (camPos.z < params.camera.m_sensorDepthWorldMin) return;
	if (camPos.z > params.camera.m_sensorDepthWorldMax) return;

	// TODO: Don't upsample so much that only minimum depth makes it through
	// Consider also using some SDF style approach to accumulate and smooth a
	// depth value between points
	const int upsample = min(UPSAMPLE_MAX-2, int(0.01 * params.camera.fx / camPos.z))+3;
	const float interval = 1.0f / float(upsample / 2);

            
    // TODO:(Nick) Check depth buffer and don't do anything if already hidden?

	// Each thread in warp takes an upsample point and updates corresponding depth buffer.
	const int lane = threadIdx.x % WARP_SIZE;
	for (int i=lane; i<upsample*upsample; i+=WARP_SIZE) {
		const float u = (i % upsample) - (upsample / 2);
		const float v = (i / upsample) - (upsample / 2);

        // Make an initial estimate of the points location
		// Use centroid depth as estimate...?
		const float3 point = params.m_viewMatrix * ftl::cuda::upsampled_point(camera.points, make_float2(float(x)+float(u)*interval, float(y)+float(v)*interval));
		const float d = point.z;

		const uint2 screenPos = params.camera.cameraToKinectScreen(point);
		const unsigned int cx = screenPos.x;//+u;
        const unsigned int cy = screenPos.y;//+v;
		if (d > params.camera.m_sensorDepthWorldMin && d < params.camera.m_sensorDepthWorldMax && cx < depth.width() && cy < depth.height()) {
			// Transform estimated point to virtual cam space and output z
			atomicMin(&depth(cx,cy), d * 1000.0f);
		}
	}
}

/*
 * Pass 1: Directly render each camera into virtual view but with no upsampling
 * for sparse points.
 */
 __global__ void dibr_merge_kernel(TextureObject<int> depth, int cam, SplatParams params) {
	const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float3 worldPos = make_float3(tex2D<float4>(camera.points, x, y));
	if (worldPos.x == MINF) return;

    // Find the virtual screen position of current point
	const float3 camPos = params.m_viewMatrix * worldPos;
	if (camPos.z < params.camera.m_sensorDepthWorldMin) return;
	if (camPos.z > params.camera.m_sensorDepthWorldMax) return;

	const float d = camPos.z;

	const uint2 screenPos = params.camera.cameraToKinectScreen(camPos);
	const unsigned int cx = screenPos.x;
	const unsigned int cy = screenPos.y;
	if (d > params.camera.m_sensorDepthWorldMin && d < params.camera.m_sensorDepthWorldMax && cx < depth.width() && cy < depth.height()) {
		// Transform estimated point to virtual cam space and output z
		atomicMin(&depth(cx,cy), d * 1000.0f);
	}
}

// ===== PASS 2 : Splat Visible Surface ========================================

/*
 * Pass 2: Determine depth buffer with enough accuracy for a visibility test in pass 2.
 * These values are also used as the actual surface estimate during rendering so should
 * at least be plane or sphere fitted if not MLS smoothed onto the actual surface.
 */
__global__ void OLD_dibr_visibility_kernel(TextureObject<int> depth, int cam, SplatParams params) {
	const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

	const int x = (blockIdx.x*blockDim.x + threadIdx.x) / WARP_SIZE;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float3 worldPos = make_float3(tex2D<float4>(camera.points, x, y));
	const float3 normal = make_float3(tex2D<float4>(camera.normal, x, y));
	if (worldPos.x == MINF) return;
    const float r = (camera.poseInverse * worldPos).z / camera.params.fx;

	// Get virtual camera ray for splat centre and backface cull if possible
	//const float3 rayOrigin = params.m_viewMatrixInverse * make_float3(0.0f,0.0f,0.0f);
	//const float3 rayDir = normalize(params.m_viewMatrixInverse * params.camera.kinectDepthToSkeleton(x,y,1.0f) - rayOrigin);
	//if (dot(rayDir, normal) > 0.0f) return;

    // Find the virtual screen position of current point
	const float3 camPos = params.m_viewMatrix * worldPos;
	if (camPos.z < params.camera.m_sensorDepthWorldMin) return;
	if (camPos.z > params.camera.m_sensorDepthWorldMax) return;
	const uint2 screenPos = params.camera.cameraToKinectScreen(camPos);

	const int upsample = min(UPSAMPLE_MAX, int((r) * params.camera.fx / camPos.z));

	// Not on screen so stop now...
	if (screenPos.x - upsample >= depth.width() || screenPos.y - upsample >= depth.height()) return;
            
    // TODO:(Nick) Check depth buffer and don't do anything if already hidden?

	// Each thread in warp takes an upsample point and updates corresponding depth buffer.
	const int lane = threadIdx.x % WARP_SIZE;
	for (int i=lane; i<upsample*upsample; i+=WARP_SIZE) {
		const float u = (i % upsample) - (upsample / 2);
		const float v = (i / upsample) - (upsample / 2);

        // Make an initial estimate of the points location
		// Use centroid depth as estimate...?
        float3 nearest = ftl::cuda::screen_centroid<1>(camera.points, make_float2(screenPos.x+u, screenPos.y+v), make_int2(x,y), params, upsample);

		// Use current points z as estimate
		//float3 nearest = params.camera.kinectDepthToSkeleton(screenPos.x+u,screenPos.y+v,camPos.z);
		
		// Or calculate upper and lower bounds for depth and do gradient
		// descent until the gradient change is too small or max iter is reached
		// and depth remains within the bounds.
		// How to find min and max depths?

        //float ld = nearest.z;

		// TODO: (Nick) Estimate depth using points plane, but needs better normals.
		//float t;
		//if (ftl::cuda::intersectPlane(normal, worldPos, rayOrigin, rayDir, t)) {
			// Plane based estimate of surface at this pixel
			//const float3 nearest = rayOrigin + rayDir * camPos.z;
			float3 output;

            // Use MLS of camera neighbor points to get more exact estimate
            // Iterate until pixel is stable on the surface.
            for (int k=0; k<MAX_ITERATIONS; ++k) {

                // TODO:(Nick) Should perhaps use points from all cameras?
                // Instead of doing each camera separately...
                // If the depth already is close then it has already been done and can skip this point
                if (ftl::cuda::mls_point_surface<1>(camera.points, make_int2(x,y), params.m_viewMatrixInverse * nearest, output, SPATIAL_SMOOTHING) <= 0.0f) {
                    /*const unsigned int cx = screenPos.x;
                    const unsigned int cy = screenPos.y;
                    if (cx < depth.width() && cy < depth.height()) {
                        atomicMax(&depth(cx,cy), 10000.0f);
                    }*/
                    break;
                }
            
				//ftl::cuda::render_depth(depth, params, output);

				output = params.m_viewMatrix * output;

                // This is essentially the SDF function f(x), only the normal should be estimated also from the weights
                //const float d = nearest.z + (normal.x*output.x + normal.y*output.y + normal.z*output.z);

				const float d = nearest.z + copysignf(0.5f*length(output - nearest), output.z - nearest.z);
				nearest = params.camera.kinectDepthToSkeleton(screenPos.x+u,screenPos.y+v,d);

                const float2 sp = params.camera.cameraToKinectScreenFloat(output);

                //if (isStable(nearest, output, params, d)) {
                //if (fabs(sp.x - float(screenPos.x+u)) < 2.0f && fabs(sp.y - float(screenPos.y+v)) < 2.0f) {
				if (length(output - nearest) <= 2.0f * params.camera.fx / camPos.z) {
                    const unsigned int cx = screenPos.x+u;
                    const unsigned int cy = screenPos.y+v;

                    if (d > params.camera.m_sensorDepthWorldMin && d < params.camera.m_sensorDepthWorldMax && cx < depth.width() && cy < depth.height()) {
                        // Transform estimated point to virtual cam space and output z
                        atomicMin(&depth(cx,cy), d * 1000.0f);
                    }
                    break;
                }

                /*if (k >= MAX_ITERATIONS-1 && length(output - nearest) <= SPATIAL_SMOOTHING) {
                    const unsigned int cx = screenPos.x+u;
                    const unsigned int cy = screenPos.y+v;
                    if (d > params.camera.m_sensorDepthWorldMin && d < params.camera.m_sensorDepthWorldMax && cx < depth.width() && cy < depth.height()) {
						//atomicMin(&depth(cx,cy), d * 1000.0f);
						printf("ERR = %f, %f\n", fabs(sp.x - float(screenPos.x+u)), fabs(sp.y - float(screenPos.y+v)));
                    }
                }*/

                //nearest = params.camera.kinectDepthToSkeleton(screenPos.x+u,screenPos.y+v,d);  // ld + (d - ld)*0.8f
                //ld = d;
			}
		//}
	}
}

// ------ Alternative for pass 2: principle surfaces ---------------------------

#define NEIGHBOR_RADIUS 1
#define MAX_NEIGHBORS ((NEIGHBOR_RADIUS*2+1)*(NEIGHBOR_RADIUS*2+1))

/*
 * Pass 2: Determine depth buffer with enough accuracy for a visibility test in pass 2.
 * These values are also used as the actual surface estimate during rendering so should
 * at least be plane or sphere fitted if not MLS smoothed onto the actual surface.
 */
 __global__ void dibr_visibility_principal_kernel(TextureObject<int> depth, int cam, SplatParams params) {
	__shared__ float3 neighborhood_cache[2*T_PER_BLOCK][MAX_NEIGHBORS];
	__shared__ int minimum[2*T_PER_BLOCK];
	__shared__ int maximum[2*T_PER_BLOCK];

	const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

	const int warp = threadIdx.x / WARP_SIZE + threadIdx.y*2;
	const int x = (blockIdx.x*blockDim.x + threadIdx.x) / WARP_SIZE;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float3 worldPos = make_float3(tex2D<float4>(camera.points, x, y));
	//const float3 normal = make_float3(tex2D<float4>(camera.normal, x, y));
	if (worldPos.x == MINF) return;
    const float r = (camera.poseInverse * worldPos).z / camera.params.fx;

	// Get virtual camera ray for splat centre and backface cull if possible
	//const float3 rayOrigin = params.m_viewMatrixInverse * make_float3(0.0f,0.0f,0.0f);
	//const float3 rayDir = normalize(params.m_viewMatrixInverse * params.camera.kinectDepthToSkeleton(x,y,1.0f) - rayOrigin);
	//if (dot(rayDir, normal) > 0.0f) return;

    // Find the virtual screen position of current point
	const float3 camPos = params.m_viewMatrix * worldPos;
	if (camPos.z < params.camera.m_sensorDepthWorldMin) return;
	if (camPos.z > params.camera.m_sensorDepthWorldMax) return;
	const uint2 screenPos = params.camera.cameraToKinectScreen(camPos);

	const int upsample = min(UPSAMPLE_MAX, int((4.0f*r) * params.camera.fx / camPos.z));

	// Not on screen so stop now...
	if (screenPos.x - upsample >= depth.width() || screenPos.y - upsample >= depth.height()) return;
            
	// TODO:(Nick) Check depth buffer and don't do anything if already hidden?
	
	// TODO:(Nick) Preload point neighbors and transform to eye
	const int lane = threadIdx.x % WARP_SIZE;
	if (lane == 0) {
		minimum[warp] = 100000000;
		maximum[warp] = -100000000;
	}

	__syncwarp();

	for (int i=lane; i<MAX_NEIGHBORS; i+=WARP_SIZE) {
		const int u = (i % (2*NEIGHBOR_RADIUS+1)) - NEIGHBOR_RADIUS;
		const int v = (i / (2*NEIGHBOR_RADIUS+1)) - NEIGHBOR_RADIUS;
		const float3 point = params.m_viewMatrix * make_float3(tex2D<float4>(camera.points, x+u, y+v));
		neighborhood_cache[warp][i] = point;

		if (length(point - camPos) <= 0.04f) {
			atomicMin(&minimum[warp], point.z*1000.0f);
			atomicMax(&maximum[warp], point.z*1000.0f);
		}
	}

	__syncwarp();
	
	const float interval = (float(maximum[warp])/1000.0f - float(minimum[warp]) / 1000.0f) / float(MAX_ITERATIONS);
	//if (y == 200) printf("interval: %f\n", interval);

	// TODO:(Nick) Find min and max depths of neighbors to estimate z bounds

	// Each thread in warp takes an upsample point and updates corresponding depth buffer.
	for (int i=lane; i<upsample*upsample; i+=WARP_SIZE) {
		const float u = (i % upsample) - (upsample / 2);
		const float v = (i / upsample) - (upsample / 2);

        // Make an initial estimate of the points location
		// Use centroid depth as estimate...?
        //float3 nearest = ftl::cuda::screen_centroid<1>(camera.points, make_float2(screenPos.x+u, screenPos.y+v), make_int2(x,y), params, upsample);

		// Use current points z as estimate
		// TODO: Use min point as estimate
		float3 nearest = params.camera.kinectDepthToSkeleton(screenPos.x+u,screenPos.y+v,float(minimum[warp])/1000.0f);
		
		// Or calculate upper and lower bounds for depth and do gradient
		// descent until the gradient change is too small or max iter is reached
		// and depth remains within the bounds.
		// How to find min and max depths?

		// TODO: (Nick) Estimate depth using points plane, but needs better normals.
		//float t;
		//if (ftl::cuda::intersectPlane(normal, worldPos, rayOrigin, rayDir, t)) {
			// Plane based estimate of surface at this pixel
			//const float3 nearest = rayOrigin + rayDir * camPos.z;

            // Use MLS of camera neighbor points to get more exact estimate
            // Iterate until pixel is stable on the surface.
            for (int k=0; k<MAX_ITERATIONS; ++k) {

                // TODO:(Nick) Should perhaps use points from all cameras?
                // Instead of doing each camera separately...
                // If the depth already is close then it has already been done and can skip this point
				const float energy = ftl::cuda::mls_point_energy<MAX_NEIGHBORS>(neighborhood_cache[warp], nearest, SPATIAL_SMOOTHING);
				
				if (energy <= 0.0f) break;
            
				//ftl::cuda::render_depth(depth, params, output);

                // This is essentially the SDF function f(x), only the normal should be estimated also from the weights
                //const float d = nearest.z + (normal.x*output.x + normal.y*output.y + normal.z*output.z);

				const float d = nearest.z;
				nearest = params.camera.kinectDepthToSkeleton(screenPos.x+u,screenPos.y+v,d+interval);
				
				if (energy >= 0.1f) {
					const unsigned int cx = screenPos.x+u;
                    const unsigned int cy = screenPos.y+v;
					if (d > params.camera.m_sensorDepthWorldMin && d < params.camera.m_sensorDepthWorldMax && cx < depth.width() && cy < depth.height()) {
                        // Transform estimated point to virtual cam space and output z
                        atomicMin(&depth(cx,cy), d * 1000.0f);
					}
					break;
				}
			}
		//}
	}
}

#define NEIGHBOR_RADIUS_2 3
#define NEIGHBOR_WINDOW ((NEIGHBOR_RADIUS_2*2+1)*(NEIGHBOR_RADIUS_2*2+1))
#define MAX_NEIGHBORS_2 32

#define FULL_MASK 0xffffffff

__device__ inline float warpMax(float e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = max(e, other);
	}
	return e;
}

__device__ inline float warpMin(float e) {
	for (int i = WARP_SIZE/2; i > 0; i /= 2) {
		const float other = __shfl_xor_sync(FULL_MASK, e, i, WARP_SIZE);
		e = min(e, other);
	}
	return e;
}

#define ENERGY_THRESHOLD 0.1f
#define SMOOTHING_MULTIPLIER_A 10.0f	// For surface search
#define SMOOTHING_MULTIPLIER_B 4.0f		// For z contribution
#define SMOOTHING_MULTIPLIER_C 4.0f		// For colour contribution


/*
 * Pass 2: Determine depth buffer with enough accuracy for a visibility test in pass 2.
 * These values are also used as the actual surface estimate during rendering so should
 * at least be plane or sphere fitted if not MLS smoothed onto the actual surface.
 *
 * This version uses a previous point render as neighbour source.
 */
 __global__ void dibr_visibility_principal_kernel2(TextureObject<int> point_in, TextureObject<int> depth, SplatParams params) {
	__shared__ float3 neighborhood_cache[2*T_PER_BLOCK][MAX_NEIGHBORS_2];
	__shared__ int minimum[2*T_PER_BLOCK];
	__shared__ int maximum[2*T_PER_BLOCK];
	__shared__ unsigned int nidx[2*T_PER_BLOCK];

	const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
	const int warp = tid / WARP_SIZE; //threadIdx.x / WARP_SIZE + threadIdx.y*2;
	const int x = (blockIdx.x*blockDim.x + threadIdx.x) / WARP_SIZE;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	// Starting point for surface minimum
	float clusterBase = params.camera.m_sensorDepthWorldMin;

	// Loop to a deeper surface if not on the first one selected...
	while (clusterBase < params.camera.m_sensorDepthWorldMax) {

	const int lane = tid % WARP_SIZE;
	if (lane == 0) {
		minimum[warp] = 100000000;
		maximum[warp] = -100000000;
		nidx[warp] = 0;
	}

	__syncwarp();

	// Search for a valid minimum neighbour
	// TODO: Should this really be minimum or the median of a depth cluster?
	// cluster median seems very hard to calculate...
	for (int i=lane; i<NEIGHBOR_WINDOW; i+=WARP_SIZE) {
		const int u = (i % (2*NEIGHBOR_RADIUS_2+1)) - NEIGHBOR_RADIUS_2;
		const int v = (i / (2*NEIGHBOR_RADIUS_2+1)) - NEIGHBOR_RADIUS_2;
		const float3 point = params.camera.kinectDepthToSkeleton(x+u, y+v, float(point_in.tex2D(x+u, y+v)) / 1000.0f);
		const float3 camPos = params.camera.kinectDepthToSkeleton(x, y, point.z);

		// If it is close enough...
		// TODO: smoothing / strength should be determined by a number of factors including:
		//     1) Depth from original source
		//     2) Colour contrast in underlying RGB
		//     3) Estimated noise levels in depth values
		if (point.z > clusterBase && point.z < params.camera.m_sensorDepthWorldMax && length(point - camPos) <= SMOOTHING_MULTIPLIER_A*(point.z / params.camera.fx)) {
			atomicMin(&minimum[warp], point.z*1000.0f);
		}
	}

	__syncwarp();

	const float minDepth = float(minimum[warp])/1000.0f;
	
	// Preload valid neighbour points from within a window. A point is valid
	// if it is within a specific distance of the minimum.
	// Also calculate the maximum at the same time.
	// TODO: Could here do a small search in each camera? This would allow all
	// points to be considered, even those masked in our depth input.
	const float3 minPos = params.camera.kinectDepthToSkeleton(x, y, minDepth);

	for (int i=lane; i<NEIGHBOR_WINDOW; i+=WARP_SIZE) {
		const int u = (i % (2*NEIGHBOR_RADIUS_2+1)) - NEIGHBOR_RADIUS_2;
		const int v = (i / (2*NEIGHBOR_RADIUS_2+1)) - NEIGHBOR_RADIUS_2;
		const float3 point = params.camera.kinectDepthToSkeleton(x+u, y+v, float(point_in.tex2D(x+u, y+v)) / 1000.0f);

		// If it is close enough...
		if (point.z > params.camera.m_sensorDepthWorldMin && point.z < params.camera.m_sensorDepthWorldMax && length(point - minPos) <= SMOOTHING_MULTIPLIER_A*(point.z / params.camera.fx)) {
			// Append to neighbour list
			//unsigned int idx = atomicInc(&nidx[warp], MAX_NEIGHBORS_2-1);
			unsigned int idx = atomicAdd(&nidx[warp], 1);
			if (idx >= MAX_NEIGHBORS_2) break;
			neighborhood_cache[warp][idx] = point;
			atomicMax(&maximum[warp], point.z*1000.0f);
		}
	}

	__syncwarp();

	const float maxDepth = float(maximum[warp])/1000.0f;
	const float interval = (maxDepth - minDepth) / float(MAX_ITERATIONS);

	if (minDepth >= params.camera.m_sensorDepthWorldMax) return;
	if (maxDepth <= params.camera.m_sensorDepthWorldMin) return;
	//if (y == 200) printf("interval: %f\n", maxDepth);

	// If all samples say same depth, then agree and return
	// TODO: Check this is valid, since small energies should be removed...
	/*if (fabs(minDepth - maxDepth) < 0.0001f) {
		if (lane == 0) {
			const unsigned int cx = x;
			const unsigned int cy = y;
			if (minDepth < params.camera.m_sensorDepthWorldMax && cx < depth.width() && cy < depth.height()) {
				// Transform estimated point to virtual cam space and output z
				atomicMin(&depth(cx,cy), minDepth * 1000.0f);
			}
		}
		return;
	}*/


	float maxenergy = -1.0f;
	float bestdepth = 0.0f;

	// Search for best or threshold energy
	for (int k=lane; k<MAX_ITERATIONS; k+=WARP_SIZE) {
		const float3 nearest = params.camera.kinectDepthToSkeleton(x,y,minDepth+float(k)*interval);
		const float myenergy = ftl::cuda::mls_point_energy<MAX_NEIGHBORS_2>(neighborhood_cache[warp], nearest, min(nidx[warp], MAX_NEIGHBORS_2), SMOOTHING_MULTIPLIER_B*(nearest.z/params.camera.fx));
		const float newenergy = warpMax(max(myenergy, maxenergy));
		bestdepth = (myenergy == newenergy) ? nearest.z : (newenergy > maxenergy) ? 0.0f : bestdepth;
		maxenergy = newenergy;
	}

	// If enough energy was found and this thread was the one that found the best
	// then output the depth that this energy occured at.
	if (bestdepth > 0.0f && maxenergy >= ENERGY_THRESHOLD) {
		//printf("E D %f %f\n", maxenergy, bestdepth);
		const unsigned int cx = x;
		const unsigned int cy = y;
		if (bestdepth > params.camera.m_sensorDepthWorldMin && bestdepth < params.camera.m_sensorDepthWorldMax && cx < depth.width() && cy < depth.height()) {
			// Transform estimated point to virtual cam space and output z
			atomicMin(&depth(cx,cy), bestdepth * 1000.0f);
			//depth(cx,cy) = bestdepth * 1000.0f;
		}
	}

	// TODO: Could the threshold depend upon the number of points? Fewer points
	// due to distance is incorrect since really there may not be fewer points
	// Perhaps the best option is to make it depend on depth ... really close
	// and really far both has lower thresholds due to point densities. Other
	// option is smoothing factor and surface distances alter with distance to
	// vary the number of points used ... smoothing factor could be a multiple
	// of pixel size at given distance. Density from original source is also
	// an influencer of smoothing factor and thresholds. Colour contrast also
	// has a weighting influence, high contrast is high certainty in the
	// disparity so such points should have a high influence over choice of
	// surface location.
	//
	// Magnitude vs dispersion factor in the energy function ...
	//   * Mag is certainty of surface location
	//   * Dispersion is how far to propagate that certainty,
	if (maxenergy >= ENERGY_THRESHOLD) return;

	// Move to next possible surface...
	clusterBase = minDepth + SMOOTHING_MULTIPLIER_B*(minDepth / params.camera.fx);

	};
}

// ===== Pass 2 and 3 : Attribute contributions ================================

__device__ inline float4 make_float4(const uchar4 &c) {
    return make_float4(c.x,c.y,c.z,c.w);
}

/*
 * Pass 2: Accumulate attribute contributions if the points pass a visibility test.
 */
__global__ void dibr_attribute_contrib_kernel(
        TextureObject<int> depth_in,
        TextureObject<float4> colour_out,
        TextureObject<float4> normal_out,
        TextureObject<float> contrib_out, int cam,
        SplatParams params) {
        
	const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

	const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
	//const int warp = tid / WARP_SIZE;
	const int x = (blockIdx.x*blockDim.x + threadIdx.x) / WARP_SIZE;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float3 worldPos = make_float3(tex2D<float4>(camera.points, x, y));
	//const float3 normal = make_float3(tex2D<float4>(camera.normal, x, y));
	if (worldPos.x == MINF) return;
    const float r = (camera.poseInverse * worldPos).z / camera.params.fx;

	const float3 camPos = params.m_viewMatrix * worldPos;
	if (camPos.z < params.camera.m_sensorDepthWorldMin) return;
	if (camPos.z > params.camera.m_sensorDepthWorldMax) return;
	const uint2 screenPos = params.camera.cameraToKinectScreen(camPos);

    const int upsample = 8; //min(UPSAMPLE_MAX, int((5.0f*r) * params.camera.fx / camPos.z));

	// Not on screen so stop now...
	if (screenPos.x >= depth_in.width() || screenPos.y >= depth_in.height()) return;
            
    // Is this point near the actual surface and therefore a contributor?
    const float d = ((float)depth_in.tex2D((int)screenPos.x, (int)screenPos.y)/1000.0f);
    //if (abs(d - camPos.z) > DEPTH_THRESHOLD) return;

    // TODO:(Nick) Should just one thread load these to shared mem?
    const float4 colour = make_float4(tex2D<uchar4>(camera.colour, x, y));
    const float4 normal = tex2D<float4>(camera.normal, x, y);

	// Each thread in warp takes an upsample point and updates corresponding depth buffer.
	const int lane = tid % WARP_SIZE;
	for (int i=lane; i<upsample*upsample; i+=WARP_SIZE) {
		const float u = (i % upsample) - (upsample / 2);
		const float v = (i / upsample) - (upsample / 2);

        // Use the depth buffer to determine this pixels 3D position in camera space
        const float d = ((float)depth_in.tex2D(screenPos.x+u, screenPos.y+v)/1000.0f);
		const float3 nearest = params.camera.kinectDepthToSkeleton((int)(screenPos.x+u),(int)(screenPos.y+v),d);

        // What is contribution of our current point at this pixel?
        const float weight = ftl::cuda::spatialWeighting(length(nearest - camPos), SMOOTHING_MULTIPLIER_C*(nearest.z/params.camera.fx));
        if (screenPos.x+u < colour_out.width() && screenPos.y+v < colour_out.height() && weight > 0.0f) {  // TODO: Use confidence threshold here
            const float4 wcolour = colour * weight;
			const float4 wnormal = normal * weight;
			
			//printf("Z %f\n", d);

            // Add this points contribution to the pixel buffer
            atomicAdd((float*)&colour_out(screenPos.x+u, screenPos.y+v), wcolour.x);
            atomicAdd((float*)&colour_out(screenPos.x+u, screenPos.y+v)+1, wcolour.y);
            atomicAdd((float*)&colour_out(screenPos.x+u, screenPos.y+v)+2, wcolour.z);
            atomicAdd((float*)&colour_out(screenPos.x+u, screenPos.y+v)+3, wcolour.w);
            atomicAdd((float*)&normal_out(screenPos.x+u, screenPos.y+v), wnormal.x);
            atomicAdd((float*)&normal_out(screenPos.x+u, screenPos.y+v)+1, wnormal.y);
            atomicAdd((float*)&normal_out(screenPos.x+u, screenPos.y+v)+2, wnormal.z);
            atomicAdd((float*)&normal_out(screenPos.x+u, screenPos.y+v)+3, wnormal.w);
            atomicAdd(&contrib_out(screenPos.x+u, screenPos.y+v), weight);
        }
	}
}

/*
 * Pass 2: Accumulate attribute contributions if the points pass a visibility test.
 */
/*__global__ void dibr_attribute_contrib_kernel(
    TextureObject<int> depth_in,
	TextureObject<uchar4> colour_out,
	TextureObject<float4> normal_out, int numcams, SplatParams params) {

    const int i = threadIdx.y*blockDim.y + threadIdx.x;
    const int bx = blockIdx.x*blockDim.x;
    const int by = blockIdx.y*blockDim.y;
    const int x = bx + threadIdx.x;
    const int y = by + threadIdx.y;
    
    for (int j=0; j<numcams; ++j) {
        const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[j];
	
        float3 worldPos = make_float3(tex2D<float4>(camera.points, x, y));
        float r = (camera.poseInverse * worldPos).z;
        //if (ftl::cuda::mls_point_surface<3>(camera.points, make_int2(x,y), worldPos, 0.02f) < 0.001f) continue;
        if (worldPos.x == MINF) continue;
        
        const float3 camPos = params.m_viewMatrix * worldPos;

        // Estimate upsample factor using ratio of source depth and output depth

		const int upsample = min(15, (int)(UPSAMPLE_FACTOR * (r / camPos.z))+1);
		const float upfactor = 2.0f / (float)(upsample);

        for (int v=0; v<upsample; ++v) {
            for (int u=0; u<upsample; ++u) {
                float3 point;
                const ftl::cuda::fragment nearest = ftl::cuda::upsampled_point(camera.points, camera.normal, camera.colour,
                    make_float2((float)x-1.0f+u*upfactor,(float)y-1.0f+v*upfactor));
                //if (ftl::cuda::mls_point_surface<3>(camera.points, make_int2(x,y), nearest, point, 0.02f) < 0.001f) continue;
                ftl::cuda::render_fragment(depth_in, normal_out, colour_out, params, nearest);
            }
        }
    }
}*/



__global__ void dibr_normalise_kernel(
        TextureObject<float4> colour_in,
        TextureObject<uchar4> colour_out,
        TextureObject<float4> normals,
        TextureObject<float> contribs) {
	const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < colour_in.width() && y < colour_in.height()) {
        const float4 colour = colour_in.tex2D((int)x,(int)y);
        const float4 normal = normals.tex2D((int)x,(int)y);
        const float contrib = contribs.tex2D((int)x,(int)y);

        if (contrib > 0.0f) {
            colour_out(x,y) = make_uchar4(colour.x / contrib, colour.y / contrib, colour.z / contrib, 0);
            normals(x,y) = normal / contrib;
        }
	}
}

void ftl::cuda::dibr(const TextureObject<int> &depth_out,
        const TextureObject<uchar4> &colour_out,
        const TextureObject<float4> &normal_out,
        const TextureObject<float> &confidence_out,
		const TextureObject<float4> &tmp_colour,
		const TextureObject<int> &tmp_depth,
        int numcams,
        const SplatParams &params,
        cudaStream_t stream) {

	const dim3 sgridSize((depth_out.width() + 2 - 1)/2, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 sblockSize(2*WARP_SIZE, T_PER_BLOCK);
    const dim3 gridSize((depth_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    clearColourKernel<<<gridSize, blockSize, 0, stream>>>(colour_out);
    ftl::cuda::clear_to_zero(confidence_out, stream);
    ftl::cuda::clear_colour(tmp_colour, stream);
    ftl::cuda::clear_colour(normal_out, stream);
	
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif

	//int i=3;

	bool noSplatting = params.m_flags & ftl::render::kNoSplatting;

	// Pass 1, gather and upsample depth maps
	if (params.m_flags & ftl::render::kNoUpsampling) {
		for (int i=0; i<numcams; ++i)
			dibr_merge_kernel<<<gridSize, blockSize, 0, stream>>>((noSplatting) ? depth_out : tmp_depth, i, params);
	} else {
		for (int i=0; i<numcams; ++i)
			dibr_merge_upsample_kernel<<<sgridSize, sblockSize, 0, stream>>>((noSplatting) ? depth_out : tmp_depth, i, params);
	}

	if (noSplatting) {
		// Pass 3, accumulate all point contributions to pixels
		for (int i=0; i<numcams; ++i)
        	dibr_attribute_contrib_kernel<<<sgridSize, sblockSize, 0, stream>>>(depth_out, tmp_colour, normal_out, confidence_out, i, params);
	} else {
		// Pass 2
		dibr_visibility_principal_kernel2<<<sgridSize, sblockSize, 0, stream>>>(tmp_depth, depth_out, params);

		// Pass 3, accumulate all point contributions to pixels
		for (int i=0; i<numcams; ++i)
        	dibr_attribute_contrib_kernel<<<sgridSize, sblockSize, 0, stream>>>(depth_out, tmp_colour, normal_out, confidence_out, i, params);
	}
	// Pass 2
	//dibr_visibility_principal_kernel2<<<sgridSize, sblockSize, 0, stream>>>(tmp_depth, depth_out, params);

    // Pass 2, merge a depth map from each camera.
	//for (int i=0; i<numcams; ++i)
    //    dibr_visibility_principal_kernel<<<sgridSize, sblockSize, 0, stream>>>(depth_out, i, params);

    // Pass 4, normalise contributions
    dibr_normalise_kernel<<<gridSize, blockSize, 0, stream>>>(tmp_colour, colour_out, normal_out, confidence_out);

	cudaSafeCall( cudaGetLastError() );
	
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

void ftl::cuda::dibr_raw(const TextureObject<int> &depth_out,
    	int numcams, const SplatParams &params, cudaStream_t stream) {

    const dim3 gridSize((depth_out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);
	
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif

	//dibr_depthmap_direct_kernel<<<gridSize, blockSize, 0, stream>>>(depth_out, numcams, params);
	cudaSafeCall( cudaGetLastError() );
	
#ifdef _DEBUG
	cudaSafeCall(cudaDeviceSynchronize());
#endif
}

