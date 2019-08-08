#ifndef _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
#define _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_

#include <ftl/depth_camera.hpp>
#include <ftl/voxel_hash.hpp>
//#include <ftl/ray_cast_util.hpp>

#include "splat_params.hpp"

namespace ftl {
namespace cuda {

__device__ inline bool intersectPlane(const float3 &n, const float3 &p0, const float3 &l0, const float3 &l, float &t) { 
    // assuming vectors are all normalized
    float denom = dot(n, l); 
    if (denom > 1e-6) {  
        t = dot(p0 - l0, n) / denom; 
        return (t >= 0); 
    } 
 
    return false; 
}

__device__ inline bool intersectPlane(const float3 &n, const float3 &p0, const float3 &l, float &t) { 
    // assuming vectors are all normalized
    float denom = dot(n, l); 
    if (denom > 1e-6) {  
        t = dot(p0, n) / denom; 
        return (t >= 0); 
    }
    return false; 
}

__device__ inline bool intersectDisk(const float3 &n, const float3 &p0, float radius, const float3 &l0, const float3 &l) { 
    float t = 0; 
    if (intersectPlane(n, p0, l0, l, t)) { 
        float3 p = l0 + l * t; 
        float3 v = p - p0; 
        float d2 = dot(v, v); 
        return (sqrt(d2) <= radius); 
        // or you can use the following optimisation (and precompute radius^2)
        // return d2 <= radius2; // where radius2 = radius * radius
     }
     return false; 
}

/**
 * Get the radius of a ray intersection with a disk.
 * @param n Normalised normal of disk.
 * @param p0 Centre of disk in camera space
 * @param l Normalised ray direction in camera space
 * @return Radius from centre of disk where intersection occurred.
 */
__device__ inline float intersectDistance(const float3 &n, const float3 &p0, const float3 &l0, const float3 &l) { 
    float t = 0; 
    if (intersectPlane(n, p0, l0, l, t)) { 
        const float3 p = l0 + l * t; 
        const float3 v = p - p0; 
        const float d2 = dot(v, v); 
        return sqrt(d2);
        // or you can use the following optimisation (and precompute radius^2)
        // return d2 <= radius2; // where radius2 = radius * radius
     }
     return PINF; 
}

/**
 * Get the radius of a ray intersection with a disk.
 * @param n Normalised normal of disk.
 * @param p0 Centre of disk in camera space
 * @param l Normalised ray direction in camera space
 * @return Radius from centre of disk where intersection occurred.
 */
__device__ inline float intersectDistance(const float3 &n, const float3 &p0, const float3 &l) { 
    float t = 0; 
    if (intersectPlane(n, p0, l, t)) { 
        const float3 p = l * t; 
        const float3 v = p - p0; 
        const float d2 = dot(v, v); 
        return sqrt(d2);
        // or you can use the following optimisation (and precompute radius^2)
        // return d2 <= radius2; // where radius2 = radius * radius
     }
     return PINF; 
}

/**
 * NOTE: Not strictly isosurface currently since it includes the internals
 * of objects up to at most truncation depth.
 */
void isosurface_point_image(const ftl::voxhash::HashData& hashData,
			const ftl::cuda::TextureObject<int> &depth,
			const ftl::render::SplatParams &params, cudaStream_t stream);

//void isosurface_point_image_stereo(const ftl::voxhash::HashData& hashData,
//		const ftl::voxhash::HashParams& hashParams,
//		const RayCastData &rayCastData, const RayCastParams &params,
//		cudaStream_t stream);

// TODO: isosurface_point_cloud

void splat_points(const ftl::cuda::TextureObject<int> &depth_in,
		const ftl::cuda::TextureObject<uchar4> &colour_in,
		const ftl::cuda::TextureObject<float4> &normal_in,
		const ftl::cuda::TextureObject<float> &depth_out,
		const ftl::cuda::TextureObject<uchar4> &colour_out, const ftl::render::SplatParams &params, cudaStream_t stream);

void dibr(const ftl::cuda::TextureObject<int> &depth_out,
		const ftl::cuda::TextureObject<uchar4> &colour_out,
		const ftl::cuda::TextureObject<float4> &normal_out,
        const ftl::cuda::TextureObject<float> &confidence_out,
        const ftl::cuda::TextureObject<float4> &tmp_colour, int numcams,
		const ftl::render::SplatParams &params, cudaStream_t stream);

/**
 * Directly render input depth maps to virtual view with clipping.
 */
void dibr_raw(const ftl::cuda::TextureObject<int> &depth_out, int numcams,
		const ftl::render::SplatParams &params, cudaStream_t stream);

void dibr(const ftl::cuda::TextureObject<float> &depth_out,
    const ftl::cuda::TextureObject<uchar4> &colour_out, int numcams, const ftl::render::SplatParams &params, cudaStream_t stream);

}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_CUDA_HPP_
