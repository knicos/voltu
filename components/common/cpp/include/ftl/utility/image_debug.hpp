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

void show_image(
	const cv::cuda::GpuMat &m,
	const std::string &name,
	float scale,
	ImageVisualisation iv);

}
}

#endif