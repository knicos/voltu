/**
 * @file voltu_cv.cpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#include <voltu/opencv.hpp>
#include <voltu/types/errors.hpp>

#include <opencv2/imgproc.hpp>

void voltu::opencv::convert(voltu::ImagePtr img, ::cv::Mat &mat)
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

void voltu::opencv::convert(voltu::ImagePtr img, ::cv::cuda::GpuMat &mat)
{
	mat = voltu::opencv::toGpuMat(img);
}

cv::cuda::GpuMat voltu::opencv::toGpuMat(voltu::ImagePtr img)
{
	voltu::ImageData data = img->getDevice();

	if (data.format == voltu::ImageFormat::kBGRA8)
	{

	}
	else if (data.format == voltu::ImageFormat::kFloat32)
	{
		return ::cv::cuda::GpuMat(
			data.height,
			data.width,
			CV_32F,
			data.data
		);
	}

	throw voltu::exceptions::NotImplemented();
}

void voltu::opencv::visualise(voltu::ImagePtr img, ::cv::Mat &mat)
{
	voltu::ImageData data = img->getHost();

	if (data.format == voltu::ImageFormat::kBGRA8)
	{
		voltu::opencv::convert(img, mat);
	}
	else if (data.format == voltu::ImageFormat::kFloat32)
	{
		::cv::Mat tmp;
		voltu::opencv::convert(img, tmp);

		float maxdepth = 8.0f;  // TODO: Get from intrinsics

		//::cv::normalize(tmp, tmp, 0, 255, ::cv::NORM_MINMAX);
		tmp.convertTo(tmp, CV_8U, 255.0f / maxdepth);
		::cv::Mat mask = tmp > 0;
		::cv::subtract(::cv::Scalar(255), tmp, tmp, mask);
		
		//#if OPENCV_VERSION >= 40102
		//cv::applyColorMap(tmp, mat, cv::COLORMAP_TURBO);
		//#else
		::cv::applyColorMap(tmp, mat, ::cv::COLORMAP_INFERNO);
		//#endif
	}
	else if (data.format == voltu::ImageFormat::kFloat16_4)
	{
		::cv::Mat tmp;
		voltu::opencv::convert(img, tmp);
		tmp.convertTo(tmp, CV_32FC4);
		tmp += 1.0f;
		tmp *= 127.0f;
		tmp.convertTo(mat, CV_8UC4);
	}
}
