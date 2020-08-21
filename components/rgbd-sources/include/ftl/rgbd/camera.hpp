#pragma once
#ifndef _FTL_RGBD_CAMERA_PARAMS_HPP_
#define _FTL_RGBD_CAMERA_PARAMS_HPP_

#include <vector_types.h>
#include <cuda_runtime.h>
#include <ftl/cuda_util.hpp>

#ifndef __CUDACC__
#include <ftl/utility/msgpack.hpp>
#include <ftl/configurable.hpp>
#endif

namespace ftl{
namespace rgbd {

enum class Projection {
	PERSPECTIVE = 0,
	ORTHOGRAPHIC = 1,
	EQUIRECTANGULAR = 2
};

typedef unsigned int capability_t;

static const capability_t kCapMovable	= 0x0001;	// A movable virtual cam
static const capability_t kCapVideo		= 0x0002;	// Is a video feed
static const capability_t kCapActive	= 0x0004;	// An active depth sensor
static const capability_t kCapStereo	= 0x0008;	// Has right RGB
static const capability_t kCapDepth		= 0x0010;	// Has depth capabilities

/**
 * All properties associated with cameras. This structure is designed to
 * operate on CPU and GPU.
 */
struct __align__(16) Camera {
	float fx;				// Focal length X
	float fy;				// Focal length Y (usually same as fx)
	float cx;				// Principle point Y
	float cy;				// Principle point Y
	unsigned int width;		// Pixel width
	unsigned int height;	// Pixel height
	float minDepth;			// Near clip in meters
	float maxDepth;			// Far clip in meters
	float baseline;			// For stereo pair
	float doffs;			// Disparity offset

	Camera scaled(int width, int height) const;

	/**
	 * Convert camera coordinates into screen coordinates.
	 */
	template <typename T> __device__ __host__ T camToScreen(const float3 &pos) const;

	/**
	 * From 3D point to 2D + Depth.
	 */
	template <Projection P> __device__ __host__ float3 project(const float3 &point) const;

	/**
	 * From 2D + Depth to 3D point.
	 */
	template <Projection P> __device__ __host__ float3 unproject(const float3 &point) const;

	/**
	 * Convert screen plus depth into camera coordinates.
	 */
	__host__ __device__ float3 screenToCam(int ux, int uy, float depth) const;

	//Eigen::Vector4f eigenScreenToCam(int ux, int uy, float depth) const;

	/**
	 * Convert screen plus depth into camera coordinates.
	 */
	__host__ __device__ float3 screenToCam(uint ux, uint uy, float depth) const;

	/**
	 * Convert screen plus depth into camera coordinates.
	 */
	__host__ __device__ float3 screenToCam(float ux, float uy, float depth) const;

	#ifndef __CUDACC__

	MSGPACK_DEFINE(fx,fy,cx,cy,width,height,minDepth,maxDepth,baseline,doffs);

	/**
	 * Make a camera struct from a configurable.
	 */
	static Camera from(ftl::Configurable*);

	cv::Mat getCameraMatrix(const cv::Size& sz={0, 0}) const;
	#endif
};

};
};

// ---- IMPLEMENTATIONS --------------------------------------------------------

template <> __device__ __host__
inline float3 ftl::rgbd::Camera::project<ftl::rgbd::Projection::EQUIRECTANGULAR>(const float3 &cam) const {
	const float l = length(cam);
	const float3 ray3d = cam / l;

    //inverse formula for spherical projection, reference Szeliski book "Computer Vision: Algorithms and Applications" p439.
    const float theta = atan2(ray3d.y,sqrt(ray3d.x*ray3d.x+ray3d.z*ray3d.z));
	const float phi = atan2(ray3d.x, ray3d.z);

	const float pi = 3.14159265f;

    //get 2D point on equirectangular map
    float x_sphere = (((phi*width)/pi+width)/2.0f);
    float y_sphere = (theta+ pi/2.0f)*height/pi;

    return make_float3(x_sphere,y_sphere, l);
};

template <> __device__ __host__
inline float3 ftl::rgbd::Camera::unproject<ftl::rgbd::Projection::EQUIRECTANGULAR>(const float3 &equi) const {
	const float pi = 3.14159265f;

	float phi = (equi.x * 2.0f - float(width)) * pi / float(width);
	float theta = (equi.y * pi / float(height)) - (pi/2.0f);

	float z = cos(theta)*cos(phi);
	float x = cos(theta)*sin(phi);
	float y = sin(theta);

    return make_float3(x*equi.z,y*equi.z,z*equi.z);
};

template <> __device__ __host__
inline float3 ftl::rgbd::Camera::project<ftl::rgbd::Projection::PERSPECTIVE>(const float3 &pos) const {
	return make_float3(
		static_cast<float>(pos.x*fx/pos.z - cx),
		static_cast<float>(pos.y*fy/pos.z - cy),
		pos.z
	);
}

template <> __device__ __host__
inline float3 ftl::rgbd::Camera::unproject<ftl::rgbd::Projection::PERSPECTIVE>(const float3 &pos) const {
	const float x = static_cast<float>((pos.x+cx) / fx);
	const float y = static_cast<float>((pos.y+cy) / fy);
	return make_float3(pos.z*x, pos.z*y, pos.z);
}

template <> __device__ __host__
inline float3 ftl::rgbd::Camera::project<ftl::rgbd::Projection::ORTHOGRAPHIC>(const float3 &pos) const {
	return make_float3(
		static_cast<float>(pos.x*fx - cx),
		static_cast<float>(pos.y*fy - cy),
		pos.z
	);
}

template <> __device__ __host__
inline float3 ftl::rgbd::Camera::unproject<ftl::rgbd::Projection::ORTHOGRAPHIC>(const float3 &pos) const {
	const float x = static_cast<float>((pos.x+cx) / fx);
	const float y = static_cast<float>((pos.y+cy) / fy);
	return make_float3(x, y, pos.z);
}

template <> __device__ __host__
inline float2 ftl::rgbd::Camera::camToScreen<float2>(const float3 &pos) const {
	return make_float2(
		static_cast<float>(pos.x*fx/pos.z - cx),
		static_cast<float>(pos.y*fy/pos.z - cy)
	);
}

template <> __device__ __host__
inline int2 ftl::rgbd::Camera::camToScreen<int2>(const float3 &pos) const {
	float2 pImage = camToScreen<float2>(pos);
	return make_int2(pImage + make_float2(0.5f, 0.5f));
}

template <> __device__ __host__
inline uint2 ftl::rgbd::Camera::camToScreen<uint2>(const float3 &pos) const {
	int2 p = camToScreen<int2>(pos);
	return make_uint2(p.x, p.y);
}

__device__ __host__
inline float3 ftl::rgbd::Camera::screenToCam(uint ux, uint uy, float depth) const {
	const float x = static_cast<float>(((float)ux+cx) / fx);
	const float y = static_cast<float>(((float)uy+cy) / fy);
	return make_float3(depth*x, depth*y, depth);
}

__device__ __host__
inline float3 ftl::rgbd::Camera::screenToCam(int ux, int uy, float depth) const {
	const float x = static_cast<float>(((float)ux+cx) / fx);
	const float y = static_cast<float>(((float)uy+cy) / fy);
	return make_float3(depth*x, depth*y, depth);
}

__device__
inline float3 ftl::rgbd::Camera::screenToCam(float ux, float uy, float depth) const {
	const float x = static_cast<float>((ux+cx) / fx);
	const float y = static_cast<float>((uy+cy) / fy);
	return make_float3(depth*x, depth*y, depth);
}

#endif
