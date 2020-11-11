/**
 * @file opencv.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/cuda.hpp>
#include <voltu/types/image.hpp>

namespace voltu
{
namespace opencv
{

void convert(voltu::ImagePtr img, ::cv::Mat &mat);

void convert(voltu::ImagePtr img, ::cv::cuda::GpuMat &mat);

::cv::cuda::GpuMat toGpuMat(voltu::ImagePtr img);

void visualise(voltu::ImagePtr img, ::cv::Mat &mat);

}

struct GpuUtilities
{
	void (*visualiseDepthEnhancement)(const voltu::ImagePtr &gt, const voltu::ImagePtr &depth_old, const voltu::ImagePtr &depth_new, const voltu::ImagePtr &colour) = nullptr;
};

extern GpuUtilities gpu;

}