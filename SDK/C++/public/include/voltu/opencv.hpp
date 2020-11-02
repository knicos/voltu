#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/cuda_types.hpp>
#include <voltu/types/image.hpp>

namespace voltu
{
namespace cv
{

void convert(voltu::ImagePtr img, ::cv::Mat &mat);

void convert(voltu::ImagePtr img, ::cv::cuda::GpuMat &mat);

void visualise(voltu::ImagePtr img, ::cv::Mat &mat);

}
}