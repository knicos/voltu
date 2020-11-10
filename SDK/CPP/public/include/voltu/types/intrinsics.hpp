/**
 * @file intrinsics.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

#include "../defines.hpp"

#include <Eigen/Eigen>

namespace voltu
{

/** Intrinsic camera paramters */
struct Intrinsics
{
	unsigned int width;
	unsigned int height;
	float principle_x;
	float principle_y;
	float focal_x;
	float focal_y;

	/** Projection matrix */
	PY_API Eigen::Matrix3d matrix();
	/** Size (width, height) */
	PY_API Eigen::Vector2i size();
};

/** Stereo camera intrinsic parameters.
 *
 * Baseline can be used to estimate depth accuracy for known depth. Assuming 1px
 * disparity accuracy, accuracy of given depth z, the accuracy of depth is
 * z^2/(f*T), where f is focal length and T baseline.
 */
struct StereoIntrinsics : public Intrinsics
{
	float baseline;
	float min_depth;
	float max_depth;
};

}
