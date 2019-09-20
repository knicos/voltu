#ifndef _FTL_CUDA_INTERSECTIONS_HPP_
#define _FTL_CUDA_INTERSECTIONS_HPP_

#ifndef PINF
#define PINF __int_as_float(0x7f800000)
#endif

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

}
}

#endif  // _FTL_CUDA_INTERSECTIONS_HPP_
