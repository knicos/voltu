/**
 * @file transformation.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once
#ifndef _FTL_TRANSFORMATION_HPP_
#define _FTL_TRANSFORMATION_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>

#include "ftl/utility/msgpack.hpp"

namespace ftl {
namespace codecs {
struct Transformation {
	Transformation() {};
	Transformation(const int &id, const cv::Vec3d &rvec, const cv::Vec3d &tvec) :
		id(id), rvec(rvec), tvec(tvec) {}

	int id;
	cv::Vec3d rvec;
	cv::Vec3d tvec;

	MSGPACK_DEFINE_ARRAY(id, rvec, tvec);

	cv::Mat rmat() const {
		cv::Mat R(cv::Size(3, 3), CV_64FC1);
		cv::Rodrigues(rvec, R);
		return R;
	}

	cv::Mat matrix() const {
		cv::Mat M = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
		rmat().copyTo(M(cv::Rect(0, 0, 3, 3)));
		M.at<double>(0, 3) = tvec[0];
		M.at<double>(1, 3) = tvec[1];
		M.at<double>(2, 3) = tvec[2];
		return M;
	}
};

}
}

#endif
