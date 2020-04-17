#include "middlebury.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

cv::Mat read_pfm(const std::string &filename) {
	cv::Mat im;
	FILE * fp;

	fp = fopen(filename.c_str(), "rb");
	char buf[32];
	int fsize;
	int width;
	int height;
	float scale;

	if (fp == NULL) { return im; }

	if (fscanf(fp, "%31s", buf) == 0 || strcmp(buf, "Pf")) { goto cleanup; }

	if (fscanf(fp, "%31s", buf) == 0) { goto cleanup; }
	width = atoi(buf);

	if (fscanf(fp, "%31s", buf) == 0) { goto cleanup; }
	height = atoi(buf);

	if (fscanf(fp, " %31s", buf) == 0) { goto cleanup; }
	scale = atof(buf);

	im.create(height, width, CV_32FC1);

	fseek(fp, 0, SEEK_END);
	fseek(fp, ftell(fp)-width*height*sizeof(float), SEEK_SET);

	for (int y = 0; y < height; y++) {
		float* im_ptr = im.ptr<float>(height-y-1);
		int nread = 0;
		do {
			nread += fread(im_ptr+nread, sizeof(float), width-nread, fp);
			if (ferror(fp)) { goto cleanup; }
		}
		while (nread != width);
	}

	cleanup:
	fclose(fp);
	return im;
}

static void parse_camera_parameters(const std::string &v, MiddEvalCalib &calib) {
	std::string mat = std::string(v.substr(1, v.size()-2));
	std::string row;

	std::vector<float> values;

	for (int _ = 0; _ < 3; _++) {
		size_t pos = mat.find(";");
		row = mat.substr(0, pos);

		std::istringstream sstr(row);
		for(std::string val; sstr >> val;) {
			values.push_back(atof(val.c_str()));
		}

		mat.erase(0, pos+1);
	}

	calib.f = values[0];
	calib.cx = values[2];
	calib.cy = values[5];
}

MiddEvalCalib read_calibration(const std::string &filename) {
	MiddEvalCalib calib;
	memset(&calib, 0, sizeof(calib));

	std::ifstream f(filename);

	for(std::string line; std::getline(f, line);) {
		auto m = line.find("=");
		if (m == std::string::npos) { continue; }
		auto k = line.substr(0, m);
		auto v = line.substr(m+1);

		if (k == "baseline") {
			calib.baseline = atof(v.c_str());
		}
		else if (k == "doffs") {
			calib.doffs = atof(v.c_str());
		}
		else if (k == "ndisp") {
			calib.ndisp = atoi(v.c_str());
		}
		else if (k == "vmin") {
			calib.vmin = atoi(v.c_str());
		}
		else if (k == "vmax") {
			calib.vmax = atoi(v.c_str());
		}
		else if (k == "width") {
			calib.width = atoi(v.c_str());
		}
		else if (k == "height") {
			calib.height = atoi(v.c_str());
		}
		else if (k == "cam0") {
			parse_camera_parameters(v, calib);
		}
	}

	return calib;
}

MiddEvalResult evaluate(const cv::Mat &disp, const cv::Mat &gt, const cv::Mat &mask, float threshold) {
	MiddEvalResult result {0.0f, 0.0f, 0.0f};
	result.threshold = threshold;

	cv::Mat diff1(disp.size(), CV_32FC1);
	cv::Mat diff2;

	cv::absdiff(disp, gt, diff1);
	diff1.setTo(0.0f, gt != gt); // NaN

	// ...? where do inf values come from; issue in subpixel interpolation?
	// doesn't seem to happen when interpolation is enabled
	diff1.setTo(0.0f, gt == INFINITY);

	float n, n_gt, n_bad, err2, err2_bad;

	// errors incl. occluded areas
	n = countNonZero(disp);
	n_gt = countNonZero(gt);
	cv::pow(diff1, 2, diff2);
	err2 = cv::sum(diff2)[0];

	result.err_total = n/n_gt;
	result.rms_total = sqrt(err2/n);

	diff2.setTo(0.0f, diff1 > threshold);
	n_bad = countNonZero(diff1 <= threshold);
	err2_bad = cv::sum(diff2)[0];
	result.err_bad = n_bad/n_gt;
	result.rms_bad = sqrt(err2_bad/n);

	// errors ignoring occlusions (mask)
	diff1.setTo(0.0f, mask != 255);
	cv::pow(diff1, 2, diff2);

	cv::Mat tmp;
	disp.copyTo(tmp);
	tmp.setTo(0.0f, mask != 255);
	n = countNonZero(tmp);
	n_gt = countNonZero(mask == 255);
	err2 = cv::sum(diff2)[0];

	result.err_nonoccl = n/n_gt;
	result.rms_nonoccl = sqrt(err2/n);

	diff2.setTo(0.0f, diff1 > threshold);
	n_bad = countNonZero(diff1 <= threshold) - countNonZero(mask != 255);
	err2_bad = cv::sum(diff2)[0];
	result.err_bad_nonoccl = n_bad/n_gt;
	result.rms_bad_nonoccl = sqrt(err2_bad/n);
	return result;
}
