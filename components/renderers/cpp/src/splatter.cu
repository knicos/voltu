#include <ftl/render/splat_params.hpp>
#include "splatter_cuda.hpp"
#include <ftl/rgbd/camera.hpp>
#include <ftl/cuda_common.hpp>

#include <ftl/cuda/weighting.hpp>
#include <ftl/cuda/intersections.hpp>
#include <ftl/cuda/warp.hpp>

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
		float3 n = make_float3(normals.tex2D((int)x,(int)y));
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

void ftl::cuda::dibr_merge(TextureObject<float4> &points, TextureObject<float4> &normals, TextureObject<int> &depth, SplatParams params, bool culling, cudaStream_t stream) {
    const dim3 gridSize((depth.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (depth.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

	if (culling) dibr_merge_kernel<true><<<gridSize, blockSize, 0, stream>>>(points, normals, depth, params);
	else dibr_merge_kernel<false><<<gridSize, blockSize, 0, stream>>>(points, normals, depth, params);
    cudaSafeCall( cudaGetLastError() );
}

//==============================================================================

__device__ inline float4 make_float4(const uchar4 &c) {
    return make_float4(c.x,c.y,c.z,c.w);
}

__device__ inline float4 make_float4(const float4 &v) {
	return v;
}

template <typename T>
__device__ inline T make();

template <>
__device__ inline uchar4 make() {
	return make_uchar4(0,0,0,0);
}

template <>
__device__ inline float4 make() {
	return make_float4(0.0f,0.0f,0.0f,0.0f);
}

template <>
__device__ inline float make() {
	return 0.0f;
}

template <typename T>
__device__ inline T make(const float4 &);

template <>
__device__ inline uchar4 make(const float4 &v) {
	return make_uchar4((int)v.x, (int)v.y, (int)v.z, (int)v.w);
}

template <>
__device__ inline float4 make(const float4 &v) {
	return v;
}

template <>
__device__ inline float make(const float4 &v) {
	return v.x;
}

/*
 * Pass 1b: Expand splats to full size and merge
 */
 template <int SEARCH_DIAMETER, typename T>
 __global__ void splat_kernel(
        //TextureObject<float4> points,       // Original 3D points
        TextureObject<float4> normals,
        TextureObject<T> in,
        TextureObject<int> depth_in,        // Virtual depth map
        TextureObject<float> depth_out,   // Accumulated output
        TextureObject<T> out,
        //ftl::rgbd::Camera camera,
        //float4x4 pose_inv,
        SplatParams params) {
        
    //const ftl::voxhash::DepthCameraCUDA &camera = c_cameras[cam];

    const int tid = (threadIdx.x + threadIdx.y * blockDim.x);
    //const int warp = tid / WARP_SIZE;
    const int x = (blockIdx.x*blockDim.x + threadIdx.x) / WARP_SIZE;
    const int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < 0 || y < 0 || x >= depth_in.width() || y >= depth_in.height()) return;

    const float3 origin = params.m_viewMatrixInverse * make_float3(0.0f);
    float3 ray = params.camera.screenToCam(x,y,1.0f);
    ray = ray / length(ray);
    const float scale = ray.z;
    ray = params.m_viewMatrixInverse.getFloat3x3() * ray;

    //float depth = 0.0f;
    //float contrib = 0.0f;
    float depth = 1000.0f;

    struct Result {
        float weight;
        float depth;
        T attr;
    };

    Result results[(SEARCH_DIAMETER*SEARCH_DIAMETER) / WARP_SIZE];

    // Each thread in warp takes an upsample point and updates corresponding depth buffer.
    const int lane = tid % WARP_SIZE;
    for (int i=lane; i<SEARCH_DIAMETER*SEARCH_DIAMETER; i+=WARP_SIZE) {
        const float u = (i % SEARCH_DIAMETER) - (SEARCH_DIAMETER / 2);
        const float v = (i / SEARCH_DIAMETER) - (SEARCH_DIAMETER / 2);

        results[i/WARP_SIZE] = {0.0f, 0.0f, make<T>()};

        // Use the depth buffer to determine this pixels 3D position in camera space
        const float d = ((float)depth_in.tex2D(x+u, y+v)/1000.0f);

        if (d < params.camera.minDepth || d > params.camera.maxDepth) continue;

        const float3 camPos = params.camera.screenToCam((int)(x+u),(int)(y+v),d);
        const float3 camPos2 = params.camera.screenToCam((int)(x),(int)(y),d);
        const float3 worldPos = params.m_viewMatrixInverse * camPos;


        // Assumed to be normalised
        float4 n = normals.tex2D((int)(x+u), (int)(y+v));

        // Does the ray intersect plane of splat?
        float t = 1000.0f;
        if (ftl::cuda::intersectPlane(make_float3(n), worldPos, origin, ray, t)) { //} && fabs(t-camPos.z) < 0.01f) {
            // Adjust from normalised ray back to original meters units
            t *= scale;
            const float3 camPos3 = params.camera.screenToCam((int)(x),(int)(y),t);
            float weight = ftl::cuda::spatialWeighting(camPos, camPos3, 2.0f*(camPos3.z/params.camera.fx));

            /* Buehler C. et al. 2001. Unstructured Lumigraph Rendering. */
            /* Orts-Escolano S. et al. 2016. Holoportation: Virtual 3D teleportation in real-time. */
            // This is the simple naive colour weighting. It might be good
            // enough for our purposes if the alignment step prevents ghosting
            // TODO: Use depth and perhaps the neighbourhood consistency in:
            //     Kuster C. et al. 2011. FreeCam: A hybrid camera system for interactive free-viewpoint video
            if (params.m_flags & ftl::render::kNormalWeightColours) weight *= n.w * n.w;
            //if (params.m_flags & ftl::render::kDepthWeightColours) weight *= ???

            if (weight <= 0.0f) continue;

            depth = min(depth, t);
            results[i/WARP_SIZE] = {weight, t, in.tex2D((int)x+u, (int)y+v)};
        }
    }

    depth = warpMin(depth);

    float adepth = 0.0f;
    float contrib = 0.0f;
    float4 attr = make_float4(0.0f);

    // Loop over results array
    for (int i=0; i<(SEARCH_DIAMETER*SEARCH_DIAMETER) / WARP_SIZE; ++i) {
        if (results[i].depth - depth < 0.04f) {
            adepth += results[i].depth * results[i].weight;
            attr += make_float4(results[i].attr) * results[i].weight;
            contrib += results[i].weight;
        }
    }

    // Sum all attributes and contributions
    adepth = warpSum(adepth);
    attr.x = warpSum(attr.x);
    attr.y = warpSum(attr.y);
    attr.z = warpSum(attr.z);
    contrib = warpSum(contrib);

    if (lane == 0 && contrib > 0.0f) {
        depth_out(x,y) = adepth / contrib;
        out(x,y) = make<T>(attr / contrib);
    }
}

template <typename T>
void ftl::cuda::splat(
        TextureObject<float4> &normals,
        TextureObject<T> &colour_in,
        TextureObject<int> &depth_in,        // Virtual depth map
        TextureObject<float> &depth_out,
        TextureObject<T> &colour_out,
        const SplatParams &params, cudaStream_t stream) {
    const dim3 gridSize((depth_in.width() + 2 - 1)/2, (depth_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(2*WARP_SIZE, T_PER_BLOCK);

    splat_kernel<8,T><<<gridSize, blockSize, 0, stream>>>(
        normals,
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
        TextureObject<uchar4> &colour_in,
        TextureObject<int> &depth_in,        // Virtual depth map
        TextureObject<float> &depth_out,
        TextureObject<uchar4> &colour_out,
		const SplatParams &params, cudaStream_t stream);
		
template void ftl::cuda::splat<float4>(
	TextureObject<float4> &normals,
	TextureObject<float4> &colour_in,
	TextureObject<int> &depth_in,        // Virtual depth map
	TextureObject<float> &depth_out,
	TextureObject<float4> &colour_out,
	const SplatParams &params, cudaStream_t stream);

template void ftl::cuda::splat<float>(
	TextureObject<float4> &normals,
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

/*
 * Pass 2: Accumulate attribute contributions if the points pass a visibility test.
 */
 template <typename T>
__global__ void dibr_attribute_contrib_kernel(
        TextureObject<T> in,				// Attribute input
        TextureObject<float4> points,       // Original 3D points
        TextureObject<int> depth_in,        // Virtual depth map
        TextureObject<T> out,			// Accumulated output
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
    const int d = depth_in.tex2D((int)screenPos.x, (int)screenPos.y);

	const T input = generateInput(in.tex2D(x, y), params, worldPos);

	//const float3 nearest = params.camera.screenToCam((int)(screenPos.x),(int)(screenPos.y),d);

	//const float l = length(nearest - camPos);
	if (d == (int)(camPos.z*1000.0f)) {
		out(screenPos.x, screenPos.y) = input;
	}
}


template <typename T>
void ftl::cuda::dibr_attribute(
        TextureObject<T> &in,
        TextureObject<float4> &points,       // Original 3D points
        TextureObject<int> &depth_in,        // Virtual depth map
        TextureObject<T> &out,   // Accumulated output
        SplatParams &params, cudaStream_t stream) {
	const dim3 gridSize((in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
	const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    dibr_attribute_contrib_kernel<<<gridSize, blockSize, 0, stream>>>(
        in,
        points,
        depth_in,
        out,
        params
    );
    cudaSafeCall( cudaGetLastError() );
}

template void ftl::cuda::dibr_attribute<uchar4>(
	ftl::cuda::TextureObject<uchar4> &in,	// Original colour image
	ftl::cuda::TextureObject<float4> &points,		// Original 3D points
	ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<uchar4> &out,	// Accumulated output
	ftl::render::SplatParams &params, cudaStream_t stream);

template void ftl::cuda::dibr_attribute<float>(
	ftl::cuda::TextureObject<float> &in,	// Original colour image
	ftl::cuda::TextureObject<float4> &points,		// Original 3D points
	ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float> &out,	// Accumulated output
	ftl::render::SplatParams &params, cudaStream_t stream);

template void ftl::cuda::dibr_attribute<float4>(
	ftl::cuda::TextureObject<float4> &in,	// Original colour image
	ftl::cuda::TextureObject<float4> &points,		// Original 3D points
	ftl::cuda::TextureObject<int> &depth_in,		// Virtual depth map
	ftl::cuda::TextureObject<float4> &out,	// Accumulated output
	ftl::render::SplatParams &params, cudaStream_t stream);

//==============================================================================

/*__global__ void dibr_normalise_kernel(
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

__global__ void dibr_normalise_kernel(
        TextureObject<float4> colour_in,
        TextureObject<float> colour_out,
        //TextureObject<float4> normals,
        TextureObject<float> contribs) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < colour_in.width() && y < colour_in.height()) {
        const float4 colour = colour_in.tex2D((int)x,(int)y);
        //const float4 normal = normals.tex2D((int)x,(int)y);
        const float contrib = contribs.tex2D((int)x,(int)y);

        if (contrib > 0.0f) {
            colour_out(x,y) = colour.x / contrib;
            //normals(x,y) = normal / contrib;
        }
    }
}

__global__ void dibr_normalise_kernel(
        TextureObject<float4> colour_in,
        TextureObject<float4> colour_out,
        //TextureObject<float4> normals,
        TextureObject<float> contribs) {
    const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    if (x < colour_in.width() && y < colour_in.height()) {
        const float4 colour = colour_in.tex2D((int)x,(int)y);
        //const float4 normal = normals.tex2D((int)x,(int)y);
        const float contrib = contribs.tex2D((int)x,(int)y);

        if (contrib > 0.0f) {
            colour_out(x,y) = make_float4(colour.x / contrib, colour.y / contrib, colour.z / contrib, 0);
            //normals(x,y) = normal / contrib;
        }
    }
}

void ftl::cuda::dibr_normalise(TextureObject<float4> &colour_in, TextureObject<uchar4> &colour_out, TextureObject<float> &contribs, cudaStream_t stream) {
    const dim3 gridSize((colour_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    dibr_normalise_kernel<<<gridSize, blockSize, 0, stream>>>(colour_in, colour_out, contribs);
    cudaSafeCall( cudaGetLastError() );
}

void ftl::cuda::dibr_normalise(TextureObject<float4> &colour_in, TextureObject<float> &colour_out, TextureObject<float> &contribs, cudaStream_t stream) {
    const dim3 gridSize((colour_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    dibr_normalise_kernel<<<gridSize, blockSize, 0, stream>>>(colour_in, colour_out, contribs);
    cudaSafeCall( cudaGetLastError() );
}

void ftl::cuda::dibr_normalise(TextureObject<float4> &colour_in, TextureObject<float4> &colour_out, TextureObject<float> &contribs, cudaStream_t stream) {
    const dim3 gridSize((colour_in.width() + T_PER_BLOCK - 1)/T_PER_BLOCK, (colour_in.height() + T_PER_BLOCK - 1)/T_PER_BLOCK);
    const dim3 blockSize(T_PER_BLOCK, T_PER_BLOCK);

    dibr_normalise_kernel<<<gridSize, blockSize, 0, stream>>>(colour_in, colour_out, contribs);
    cudaSafeCall( cudaGetLastError() );
}*/
