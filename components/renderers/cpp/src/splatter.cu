#include <ftl/render/splat_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

#include <ftl/cuda/weighting.hpp>
#include <ftl/cuda/intersections.hpp>
#include <ftl/cuda/warp.hpp>
#include <ftl/cuda/makers.hpp>

#define T_PER_BLOCK 8
#define UPSAMPLE_FACTOR 1.8f
#define WARP_SIZE 32
#define DEPTH_THRESHOLD 0.05f
#define UPSAMPLE_MAX 60
#define MAX_ITERATIONS 32  // Note: Must be multiple of 32
#define SPATIAL_SMOOTHING 0.005f

#define ENERGY_THRESHOLD 0.1f
#define SMOOTHING_MULTIPLIER_A 10.0f	// For surface search
#define SMOOTHING_MULTIPLIER_B 4.0f		// For z contribution
#define SMOOTHING_MULTIPLIER_C 2.0f		// For colour contribution

#define ACCUM_DIAMETER 8

using ftl::cuda::TextureObject;
using ftl::render::SplatParams;
using ftl::cuda::warpMin;
using ftl::cuda::warpSum;

/*
 * Pass 1: Directly render each camera into virtual view but with no upsampling
 * for sparse points.
 */
 template <bool CULLING>
 __global__ void dibr_merge_kernel(TextureObject<float4> points,
		TextureObject<float4> normals,
		TextureObject<int> depth, SplatParams params) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float4 worldPos = points.tex2D(x, y);
	if (worldPos.x == MINF || (!(params.m_flags & ftl::render::kShowDisconMask) && worldPos.w < 0.0f)) return;

	// Compile time enable/disable of culling back facing points
	if (CULLING) {
		float3 ray = params.m_viewMatrixInverse.getFloat3x3() * params.camera.screenToCam(x,y,1.0f);
        ray = ray / length(ray);
        float4 n4 = normals.tex2D((int)x,(int)y);
        if (n4.w == 0.0f) return;
		float3 n = make_float3(n4);
		float l = length(n);
		if (l == 0) {
			return;
		}
		n /= l;

		const float facing = dot(ray, n);
		if (facing <= 0.0f) return;
	}

    // Find the virtual screen position of current point
	const float3 camPos = params.m_viewMatrix * make_float3(worldPos);
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

/*
 * Pass 1: Directly render each camera into virtual view but with no upsampling
 * for sparse points.
 */
 __global__ void dibr_merge_kernel(TextureObject<float4> points,
		TextureObject<int> depth, SplatParams params) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float4 worldPos = points.tex2D(x, y);
	if (worldPos.x == MINF || (!(params.m_flags & ftl::render::kShowDisconMask) && worldPos.w < 0.0f)) return;

    // Find the virtual screen position of current point
	const float3 camPos = params.m_viewMatrix * make_float3(worldPos);
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

/*
 * Pass 1: Directly render each camera into virtual view but with no upsampling
 * for sparse points.
 */
 __global__ void dibr_merge_kernel(TextureObject<float> depth,
		TextureObject<int> depth_out,
		float4x4 transform,
		ftl::rgbd::Camera cam,
		SplatParams params) {
	const int x = blockIdx.x*blockDim.x + threadIdx.x;
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float d0 = depth.tex2D(x, y);
	if (d0 <= cam.minDepth || d0 >= cam.maxDepth) return;

	const float3 camPos = transform * cam.screenToCam(x,y,d0);
	//if (worldPos.x == MINF || (!(params.m_flags & ftl::render::kShowDisconMask) && worldPos.w < 0.0f)) return;

	// Find the virtual screen position of current point
	//const float3 camPos = params.m_viewMatrix * make_float3(worldPos);
	//if (camPos.z < params.camera.minDepth) return;
	//if (camPos.z > params.camera.maxDepth) return;

	const float d = camPos.z;

	const uint2 screenPos = params.camera.camToScreen<uint2>(camPos);
	const unsigned int cx = screenPos.x;
	const unsigned int cy = screenPos.y;
	if (d > params.camera.minDepth && d < params.camera.maxDepth && cx < depth.width() && cy < depth.height()) {
		// Transform estimated point to virtual cam space and output z
		atomicMin(&depth_out(cx,cy), d * 100000.0f);
	}
}

void ftl::cuda::dibr_merge(TextureObject<float4> &points, TextureObject<float4> &normals, TextureObject<int> &depth, SplatParams params, bool culling, cudaStream_t stream) {
    const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	if (culling) dibr_merge_kernel<true><<<gridSize, blockSize, 0, stream>>>(points, normals, depth, params);
	else dibr_merge_kernel<false><<<gridSize, blockSize, 0, stream>>>(points, normals, depth, params);
    cudaSafeCall( cudaGetLastError() );
}

void ftl::cuda::dibr_merge(TextureObject<float4> &points, TextureObject<int> &depth, SplatParams params, cudaStream_t stream) {
    const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	dibr_merge_kernel<<<gridSize, blockSize, 0, stream>>>(points, depth, params);
    cudaSafeCall( cudaGetLastError() );
}

void ftl::cuda::dibr_merge(TextureObject<float> &depth, TextureObject<int> &depth_out, const float4x4 &transform, const ftl::rgbd::Camera &cam, SplatParams params, cudaStream_t stream) {
    const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	dibr_merge_kernel<<<gridSize, blockSize, 0, stream>>>(depth, depth_out, transform, cam, params);
    cudaSafeCall( cudaGetLastError() );
}

//==============================================================================


/*
 * Pass 1b: Expand splats to full size and merge
 */
 template <int SEARCH_RADIUS, typename T>
 __global__ void splat_kernel(
        //TextureObject<float4> points,       // Original 3D points
        TextureObject<float4> normals,
        TextureObject<float> density,
        TextureObject<T> in,
        TextureObject<int> depth_in,        // Virtual depth map
        TextureObject<float> depth_out,   // Accumulated output
        TextureObject<T> out,
        //ftl::rgbd::Camera camera,
        //float4x4 pose_inv,
        SplatParams params) {
        
    //const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

    //const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
    //const int warp = tid / WARP_SIZE;
    const int x = blockIdx.x*blockDim.x + threadIdx.x;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || y < 0 || x >= depth_in.width() || y >= depth_in.height()) return;

    //const float3 origin = params.m_viewMatrixInverse * make_float3(0.0f);
    float3 ray = params.camera.screenToCam(x,y,1.0f);
    ray = ray / length(ray);
    const float scale = ray.z;
    //ray = params.m_viewMatrixInverse.getFloat3x3() * ray;

    //float depth = 0.0f;
    //float contrib = 0.0f;
    float depth = 1000.0f;
    //float pdepth = 1000.0f;

    struct Result {
        float weight;
        float depth;
    };

    Result results[2*SEARCH_RADIUS+1][2*SEARCH_RADIUS+1];

    // Each thread in warp takes an upsample point and updates corresponding depth buffer.
    //const int lane = tid % WARP_SIZE;
    //for (int i=lane; i<SEARCH_DIAMETER*SEARCH_DIAMETER; i+=WARP_SIZE) {
    //    const float u = (i % SEARCH_DIAMETER) - (SEARCH_DIAMETER / 2);
    //    const float v = (i / SEARCH_DIAMETER) - (SEARCH_DIAMETER / 2);
    for (int v=-SEARCH_RADIUS; v<=SEARCH_RADIUS; ++v) {
    for (int u=-SEARCH_RADIUS; u<=SEARCH_RADIUS; ++u) {

        results[v+SEARCH_RADIUS][u+SEARCH_RADIUS] = {0.0f, 1000.0f};

        // Use the depth buffer to determine this pixels 3D position in camera space
        const float d = ((float)depth_in.tex2D(x+u, y+v)/1000.0f);

        const float3 n = make_float3(normals.tex2D((int)(x)+u, (int)(y)+v));
        const float dens = density.tex2D((int)(x)+u, (int)(y)+v);

        if (d < params.camera.minDepth || d > params.camera.maxDepth) continue;

        const float3 camPos = params.camera.screenToCam((int)(x)+u,(int)(y)+v,d);
        //const float3 camPos2 = params.camera.screenToCam((int)(x),(int)(y),d);
        //const float3 worldPos = params.m_viewMatrixInverse * camPos;


        
		//if (length(make_float3(n)) == 0.0f) printf("BAD NORMAL\n");

        // Does the ray intersect plane of splat?
        float t = 1000.0f;
        const float r = ftl::cuda::intersectDistance(n, camPos, make_float3(0.0f), ray, t);
        //if (r != PINF) { //} && fabs(t-camPos.z) < 0.01f) {
            // Adjust from normalised ray back to original meters units
            t *= scale;
            float weight = ftl::cuda::weighting(r, dens/params.camera.fx); // (1.0f/params.camera.fx) / (t/params.camera.fx)

            /* Buehler C. et al. 2001. Unstructured Lumigraph Rendering. */
            /* Orts-Escolano S. et al. 2016. Holoportation: Virtual 3D teleportation in real-time. */
            // This is the simple naive colour weighting. It might be good
            // enough for our purposes if the alignment step prevents ghosting
            // TODO: Use depth and perhaps the neighbourhood consistency in:
            //     Kuster C. et al. 2011. FreeCam: A hybrid camera system for interactive free-viewpoint video
            //if (params.m_flags & ftl::render::kNormalWeightColours) weight *= n.w * n.w;
            //if (params.m_flags & ftl::render::kDepthWeightColours) weight *= ???

            if (weight <= 0.0f) continue;

            depth = min(depth, t);
            results[v+SEARCH_RADIUS][u+SEARCH_RADIUS] = {weight, t};
        //}
    }
    }

    //depth = warpMin(depth);
    //pdepth = warpMin(pdepth);

    float adepth = 0.0f;
    float contrib = 0.0f;
    float4 attr = make_float4(0.0f);

    // Loop over results array
    for (int v=-SEARCH_RADIUS; v<=SEARCH_RADIUS; ++v) {
    for (int u=-SEARCH_RADIUS; u<=SEARCH_RADIUS; ++u) {
        auto &result = results[v+SEARCH_RADIUS][u+SEARCH_RADIUS];
        float s = ftl::cuda::weighting(fabs(result.depth - depth), 0.04f);
        //if (result.depth - depth < 0.04f) {
            adepth += result.depth * result.weight * s;
            attr += make_float4(in.tex2D((int)x+u, (int)y+v)) * result.weight * s;
            contrib += result.weight * s;
        //}
    }
    }

    // Sum all attributes and contributions
    //adepth = warpSum(adepth);
    //attr.x = warpSum(attr.x);
    //attr.y = warpSum(attr.y);
    //attr.z = warpSum(attr.z);
    //contrib = warpSum(contrib);

    if (contrib > 0.0f) {
        depth_out(x,y) = adepth / contrib;
        out(x,y) = make<T>(attr / contrib);
    }
}

template <typename T>
void ftl::cuda::splat(
        TextureObject<float4> &normals,
        TextureObject<float> &density,
        TextureObject<T> &colour_in,
        TextureObject<int> &depth_in,        // Virtual depth map
        TextureObject<float> &depth_out,
        TextureObject<T> &colour_out,
        const SplatParams &params, cudaStream_t stream) {
            const dim3 gridSize((depth_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
            const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    splat_kernel<4,T><<<gridSize, blockSize, 0, stream>>>(
        normals,
        density,
        colour_in,
        depth_in,
        depth_out,
        colour_out,
        params
    );
    cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::splat<uchar4>(
        TextureObject<float4> &normals,
        TextureObject<float> &density,
        TextureObject<uchar4> &colour_in,
        TextureObject<int> &depth_in,        // Virtual depth map
        TextureObject<float> &depth_out,
        TextureObject<uchar4> &colour_out,
		const SplatParams &params, cudaStream_t stream);
		
template void ftl::cuda::splat<float4>(
    TextureObject<float4> &normals,
    TextureObject<float> &density,
	TextureObject<float4> &colour_in,
	TextureObject<int> &depth_in,        // Virtual depth map
	TextureObject<float> &depth_out,
	TextureObject<float4> &colour_out,
	const SplatParams &params, cudaStream_t stream);

template void ftl::cuda::splat<float>(
    TextureObject<float4> &normals,
    TextureObject<float> &density,
	TextureObject<float> &colour_in,
	TextureObject<int> &depth_in,        // Virtual depth map
	TextureObject<float> &depth_out,
	TextureObject<float> &colour_out,
	const SplatParams &params, cudaStream_t stream);

//==============================================================================

template <typename T>
__device__ inline T generateInput(const T &in, const SplatParams &params, const float4 &worldPos) {
	return in;
}

template <>
__device__ inline uchar4 generateInput(const uchar4 &in, const SplatParams &params, const float4 &worldPos) {
	return (params.m_flags & ftl::render::kShowDisconMask && worldPos.w < 0.0f) ?
		make_uchar4(0,0,255,255) :  // Show discontinuity mask in red
		in;
}

template <typename A, typename B>
__device__ inline B weightInput(const A &in, float weight) {
	return in * weight;
}

template <>
__device__ inline float4 weightInput(const uchar4 &in, float weight) {
	return make_float4(
		(float)in.x * weight,
		(float)in.y * weight,
		(float)in.z * weight,
		(float)in.w * weight);
}

template <typename T>
__device__ inline void accumulateOutput(TextureObject<T> &out, TextureObject<float> &contrib, const uint2 &pos, const T &in, float w) {
	atomicAdd(&out(pos.x, pos.y), in);
	atomicAdd(&contrib(pos.x, pos.y), w);
} 

template <>
__device__ inline void accumulateOutput(TextureObject<float4> &out, TextureObject<float> &contrib, const uint2 &pos, const float4 &in, float w) {
	atomicAdd((float*)&out(pos.x, pos.y), in.x);
	atomicAdd(((float*)&out(pos.x, pos.y))+1, in.y);
	atomicAdd(((float*)&out(pos.x, pos.y))+2, in.z);
	atomicAdd(((float*)&out(pos.x, pos.y))+3, in.w);
	atomicAdd(&contrib(pos.x, pos.y), w);
} 

/*
 * Pass 2: Accumulate attribute contributions if the points pass a visibility test.
 */
 template <typename A, typename B>
__global__ void dibr_attribute_contrib_kernel(
        TextureObject<A> in,				// Attribute input
        TextureObject<float4> points,       // Original 3D points
        TextureObject<int> depth_in,        // Virtual depth map
		TextureObject<B> out,			// Accumulated output
		TextureObject<float> contrib,
        SplatParams params) {
        
	const int x = (blockIdx.x*blockDim.x + threadIdx.x);
	const int y = blockIdx.y*blockDim.y + threadIdx.y;

	const float4 worldPos = points.tex2D(x, y);
	if (worldPos.x == MINF || (!(params.m_flags & ftl::render::kShowDisconMask) && worldPos.w < 0.0f)) return;

	const float3 camPos = params.m_viewMatrix * make_float3(worldPos);
	if (camPos.z < params.camera.minDepth) return;
	if (camPos.z > params.camera.maxDepth) return;
	const uint2 screenPos = params.camera.camToScreen<uint2>(camPos);

	// Not on screen so stop now...
	if (screenPos.x >= depth_in.width() || screenPos.y >= depth_in.height()) return;
            
    // Is this point near the actual surface and therefore a contributor?
    const float d = (float)depth_in.tex2D((int)screenPos.x, (int)screenPos.y) / 1000.0f;

	const A input = generateInput(in.tex2D(x, y), params, worldPos);
	const float weight = ftl::cuda::weighting(fabs(camPos.z - d), 0.02f);
	const B weighted = make<B>(input) * weight; //weightInput(input, weight);

	if (weight > 0.0f) {
		accumulateOutput(out, contrib, screenPos, weighted, weight);
		//out(screenPos.x, screenPos.y) = input;
	}
}


template <typename A, typename B>
void ftl::cuda::dibr_attribute(
        TextureObject<A> &in,
        TextureObject<float4> &points,       // Original 3D points
        TextureObject<int> &depth_in,        // Virtual depth map
		TextureObject<B> &out,   // Accumulated output
		TextureObject<float> &contrib,
        SplatParams &params, cudaStream_t stream) {
	const dim3 gridSize((in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    dibr_attribute_contrib_kernel<<<gridSize, blockSize, 0, stream>>>(
        in,
        points,
        depth_in,
		out,
		contrib,
        params
    );
    cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::dibr_attribute(
	ftl::cuda::TextureObject<uchar4> &in,	// Original colour image
	ftl::cuda::TextureObject<float4> &points,		// Original 3D points
	ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float4> &out,	// Accumulated output
	ftl::cuda::TextureObject<float> &contrib,
	ftl::render::SplatParams &params, cudaStream_t stream);

template void ftl::cuda::dibr_attribute(
	ftl::cuda::TextureObject<float> &in,	// Original colour image
	ftl::cuda::TextureObject<float4> &points,		// Original 3D points
	ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float> &out,	// Accumulated output
	ftl::cuda::TextureObject<float> &contrib,
	ftl::render::SplatParams &params, cudaStream_t stream);

template void ftl::cuda::dibr_attribute(
	ftl::cuda::TextureObject<float4> &in,	// Original colour image
	ftl::cuda::TextureObject<float4> &points,		// Original 3D points
	ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float4> &out,	// Accumulated output
	ftl::cuda::TextureObject<float> &contrib,
	ftl::render::SplatParams &params, cudaStream_t stream);

//==============================================================================

template <typename A, typename B>
__global__ void dibr_normalise_kernel(
        TextureObject<A> in,
        TextureObject<B> out,
        TextureObject<float> contribs) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < in.width() && y < in.height()) {
		const float contrib = contribs.tex2D((int)x,(int)y);
        const A a = in.tex2D((int)x,(int)y);
        //const float4 normal = normals.tex2D((int)x,(int)y);

		//out(x,y) = (contrib == 0.0f) ? make<B>(a) : make<B>(a / contrib);

        if (contrib > 0.0f) {
            out(x,y) = make<B>(a / contrib);
            //normals(x,y) = normal / contrib;
        }
    }
}

template <typename A, typename B>
void ftl::cuda::dibr_normalise(TextureObject<A> &in, TextureObject<B> &out, TextureObject<float> &contribs, cudaStream_t stream) {
    const dim3 gridSize((in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    dibr_normalise_kernel<<<gridSize, blockSize, 0, stream>>>(in, out, contribs);
    cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::dibr_normalise<float4,uchar4>(TextureObject<float4> &in, TextureObject<uchar4> &out, TextureObject<float> &contribs, cudaStream_t stream);
template void ftl::cuda::dibr_normalise<float,float>(TextureObject<float> &in, TextureObject<float> &out, TextureObject<float> &contribs, cudaStream_t stream);
template void ftl::cuda::dibr_normalise<float4,float4>(TextureObject<float4> &in, TextureObject<float4> &out, TextureObject<float> &contribs, cudaStream_t stream);


// ===== Show bad colour normalise =============================================

__global__ void show_missing_colour_kernel(
        TextureObject<float> depth,
        TextureObject<uchar4> out,
		TextureObject<float> contribs,
		uchar4 bad_colour,
		ftl::rgbd::Camera cam) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < out.width() && y < out.height()) {
		const float contrib = contribs.tex2D((int)x,(int)y);
		const float d = depth.tex2D(x,y);

		if (contrib < 0.0000001f && d > cam.minDepth && d < cam.maxDepth) {
			out(x,y) = bad_colour;
		}
    }
}

void ftl::cuda::show_missing_colour(
		TextureObject<float> &depth,
		TextureObject<uchar4> &out,
		TextureObject<float> &contribs,
		uchar4 bad_colour,
		const ftl::rgbd::Camera &cam,
		cudaStream_t stream) {
    const dim3 gridSize((out.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (out.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    show_missing_colour_kernel<<<gridSize, blockSize, 0, stream>>>(depth, out, contribs, bad_colour, cam);
    cudaSafeCall( cudaGetLastError() );
}
