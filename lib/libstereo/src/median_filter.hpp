#pragma once

#include <opencv2/core/core.hpp>
#include "array2d.hpp"

void median_filter(cv::InputArray in, cv::OutputArray out);
void median_filter(Array2D<float> &in, cv::OutputArray out);
