/**
 * @file image_debug.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_UTILITY_IMAGE_DEBUG_HPP_
#define _FTL_UTILITY_IMAGE_DEBUG_HPP_

namespace ftl {
namespace utility {

enum class ImageVisualisation {
	RAW_COLOUR,
	RAW_GRAY,
	NORMALISED_FLOAT,
	HEAT_MAPPED
};

/**
 * Display a GpuMat in a window with a particular colorisation.
 * 
 * @param m GPU Mat input
 * @param name OpenCV window name.
 * @param scale Image resolution scale, 1 = full resolution
 * @param iv How to colour convert the image
 */
void show_image(
	const cv::cuda::GpuMat &m,
	const std::string &name,
	float scale,
	ImageVisualisation iv);

}
}

#endif