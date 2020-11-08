/**
 * @file intrinsics.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

namespace voltu
{

struct Intrinsics
{
	unsigned int width;
	unsigned int height;
	float principle_x;
	float principle_y;
	float focal_x;
	float focal_y;
};

struct StereoIntrinsics : public Intrinsics
{
	float baseline;
	float doffs;
	float min_depth;
	float max_depth;
};

}
