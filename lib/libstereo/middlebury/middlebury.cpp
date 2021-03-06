#include "middlebury.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

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

	if (calib.vmax == 0) {
		calib.vmax = calib.ndisp;
	}

	return calib;
}

MiddleburyData load_input(const fs::path &path) {
	cv::Mat imL;
	cv::Mat imR;
	cv::Mat gtL;
	cv::Mat maskL;

	imL = cv::imread(path/"im0.png");
	imR = cv::imread(path/"im1.png");
	gtL = read_pfm(path/"disp0.pfm");
	if (gtL.empty()) {
		gtL = read_pfm(path/std::string("disp0GT.pfm"));
	}
	if (gtL.empty()) {
		gtL.create(imL.size(), CV_32FC1);
		gtL.setTo(cv::Scalar(0.0f));
	}

	maskL = cv::imread(path/std::string("mask0nocc.png"), cv::IMREAD_GRAYSCALE);
	if (maskL.empty()) {
		maskL.create(imL.size(), CV_8UC1);
		maskL.setTo(cv::Scalar(255));
	}

	auto calib = read_calibration(path/std::string("calib.txt"));
	if (imL.empty() || imR.empty() || gtL.empty() || maskL.empty()) {
		throw std::exception();
	}

	std::string name = path.filename() == "." ?
		path.parent_path().filename().c_str() :
		path.filename().c_str();

	return {name, imL, imR, gtL, maskL, calib};
}

MiddleburyData load_input(const std::string &path) {
	return load_input(fs::path(path));
}

std::vector<MiddleburyData> load_directory(const std::string &path) {
	std::vector<MiddleburyData> res;
	for(auto& p: fs::directory_iterator(path)) {
		try {
			res.push_back(load_input(p.path()));
		}
		catch (...) {}
	}
	return res;
}

MiddEvalResult evaluate(const cv::Mat &disp, const cv::Mat &gtdisp, const cv::Mat &mask, float threshold) {
	MiddEvalResult result {0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	if (gtdisp.type() != CV_32FC1) { throw std::exception(); }
	if (disp.type() != CV_32FC1) { throw std::exception(); }
	if (!mask.empty() && mask.type() != CV_8UC1) { throw std::exception(); }

	using std::min;
	using std::max;

	bool usemask = !mask.empty();

	result.threshold = threshold;
	int &n = result.n;
	int &bad = result.bad;
	int good = 0;
	int &invalid = result.invalid;

	float &mse_good = result.mse_good;
	float &mse_bad = result.mse_bad;
	float &mse_total = result.mse_total;
	float serr = 0;

	// evaldisp.cpp from middlebury SDK

	for (int y = 0; y < gtdisp.rows; y++) {
	for (int x = 0; x < gtdisp.cols; x++) {
		float gt = gtdisp.at<float>(y, x, 0);
		if (gt == INFINITY) // unknown
		continue;

		float d = disp.at<float>(y, x, 0);
		int valid = (d != INFINITY && d != 0.0f); // added 0.0f
		/*if (valid) {
			float maxd = maxdisp; // max disp range
			d = max(0, min(maxd, d)); // clip disps to max disp range
		}*/
		/*if (valid && rounddisp) { d = round(d); }*/
		float err = fabs(d - gt);

		if (usemask && mask.at<uchar>(y, x, 0) != 255) {} // don't evaluate pixel
		else {
			n++;
			if (valid) {
				serr += err;
				mse_total += err*err;

				if (err > threshold) {
					bad++;
					mse_bad += err*err;
				}
				else {
					mse_good += err*err;
					good++;
				}
			} else {// invalid (i.e. hole in sparse disp map)
				invalid++;
			}
		}
	}
	}

	mse_bad = mse_bad/float(bad);
	mse_good = mse_good/float(good);
	mse_total = mse_total/float(n - invalid);
	result.avgerr = serr/float(n - invalid);
	result.err_bad = float(bad)/float(n);
	result.err_invalid = float(invalid)/float(n);
	result.err_total = float(bad+invalid)/float(n);
	return result;
}

void add_noise(cv::Mat &im, double stddev, double mean) {
	cv::Mat noise = cv::Mat(im.size(),CV_64FC3);
	cv::randn(noise, mean, stddev);

	cv::Mat tmp;
	im.convertTo(tmp, CV_64FC3, 1.0/255.0);
	tmp += noise;
	cv::normalize(tmp, im, 0, 255, cv::NORM_MINMAX, CV_8UC3);
}
