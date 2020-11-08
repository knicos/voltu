/**
 * @file voltu_cv.cpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#include <voltu/opencv.hpp>
#include <voltu/types/errors.hpp>

#include <opencv2/imgproc.hpp>

void voltu::cv::convert(voltu::ImagePtr img, ::cv::Mat &mat)
{
	voltu::ImageData data = img->getHost();

	if (data.format == voltu::ImageFormat::kBGRA8)
	{
		mat = ::cv::Mat(data.height, data.width, CV_8UC4, data.data);
	}
	else if (data.format == voltu::ImageFormat::kFloat32)
	{
		mat = ::cv::Mat(data.height, data.width, CV_32FC1, data.data);
	}
	else if (data.format == voltu::ImageFormat::kFloat16_4)
	{
		mat = ::cv::Mat(data.height, data.width, CV_16FC4, data.data);
	}
	else
	{
		mat = ::cv::Mat();
	}
}

void voltu::cv::convert(voltu::ImagePtr img, ::cv::cuda::GpuMat &mat)
{
	throw voltu::exceptions::NotImplemented();
}

void voltu::cv::visualise(voltu::ImagePtr img, ::cv::Mat &mat)
{
	voltu::ImageData data = img->getHost();

	if (data.format == voltu::ImageFormat::kBGRA8)
	{
		voltu::cv::convert(img, mat);
	}
	else if (data.format == voltu::ImageFormat::kFloat32)
	{
		::cv::Mat tmp;
		voltu::cv::convert(img, tmp);

		::cv::normalize(tmp, tmp, 0, 255, ::cv::NORM_MINMAX);
		tmp.convertTo(tmp, CV_8U);
		
		//#if OPENCV_VERSION >= 40102
		//cv::applyColorMap(tmp, mat, cv::COLORMAP_TURBO);
		//#else
		::cv::applyColorMap(tmp, mat, ::cv::COLORMAP_INFERNO);
		//#endif
	}
	else if (data.format == voltu::ImageFormat::kFloat16_4)
	{
		::cv::Mat tmp;
		voltu::cv::convert(img, tmp);
		tmp.convertTo(tmp, CV_32FC4);
		tmp += 1.0f;
		tmp *= 127.0f;
		tmp.convertTo(mat, CV_8UC4);
	}
}
