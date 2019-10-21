#include <openvr/openvr.h>
#include <Eigen/Eigen>
#include <openvr/openvr.h>

/* @brief	Calculate (pinhole camera) intrinsic matrix from OpenVR parameters
 * @param	Tangent of left half angle (negative) from center view axis
 * @param	Tangent of right half angle from center view axis
 * @param	Tangent of top half angle (negative) from center view axis
 * @param	Tangent of bottom half angle from center view axis
 * @param	Image width
 * @param	Image height
 * 
 * Parameters are provided by IVRSystem::GetProjectionRaw and
 * IVRSystem::GetRecommendedRenderTargetSize.
 * 
 * tanx1 = x1 / fx		(1)
 * tanx2 = x2 / fy		(2)
 * x1 + x2 = size_x		(3)
 * 
 * :. fx = size_x / (-tanx1 + tanx2)
 * 
 * fy can be calculated in same way
 */
Eigen::Matrix3d getCameraMatrix(const double tanx1,
								const double tanx2,
								const double tany1,
								const double tany2,
								const double size_x,
								const double size_y);

/*
 * @brief	Same as above, but uses given IVRSystem and eye.
 */
Eigen::Matrix3d getCameraMatrix(vr::IVRSystem *vr, const vr::Hmd_Eye &eye);


static inline Eigen::Matrix4d ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix34_t &matPose )
{
	Eigen::Matrix4d matrixObj;
	matrixObj <<
		matPose.m[0][0], matPose.m[0][1], matPose.m[0][2], matPose.m[0][3],
		matPose.m[1][0], matPose.m[1][1], matPose.m[1][2], matPose.m[1][3],
		matPose.m[2][0], matPose.m[2][1], matPose.m[2][2], matPose.m[2][3],
					0.0,			 0.0,			  0.0,			   1.0;
	return matrixObj;
}

static inline Eigen::Matrix4d ConvertSteamVRMatrixToMatrix4( const vr::HmdMatrix44_t &matPose )
{
	Eigen::Matrix4d matrixObj;
	matrixObj <<
		matPose.m[0][0], matPose.m[0][1], matPose.m[0][2], matPose.m[0][3],
		matPose.m[1][0], matPose.m[1][1], matPose.m[1][2], matPose.m[1][3],
		matPose.m[2][0], matPose.m[2][1], matPose.m[2][2], matPose.m[2][3],
		matPose.m[3][0], matPose.m[3][1], matPose.m[3][2], matPose.m[3][3];
	return matrixObj;
}