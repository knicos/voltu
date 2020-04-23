#include <opencv2/opencv.hpp>

#include <vector>
#include <map>
#include <chrono>

#include "middlebury.hpp"
#include "stereo.hpp"

#include "../../components/common/cpp/include/ftl/config.h"

/**
 * @param   disp    disparity map
 * @param   out     output parameter
 * @param   ndisp   number of disparities
 *
 * If ndisp is greater than zero, color mapping is scaled to range [0, ndisp],
 * otherwise scaling is automatically set to [0, max(disp)].
 */
void colorize(const cv::Mat &disp, cv::Mat &out, int ndisp=-1) {
	double dmin, dmax;
	cv::minMaxLoc(disp, &dmin, &dmax);
	cv::Mat dispf, dispc;
	disp.convertTo(dispf, CV_32FC1);
	dispf.convertTo(dispc, CV_8UC1, (1.0f / (ndisp > 0 ? (float) ndisp : dmax)) * 255.0f);

	#if OPENCV_VERSION >= 40102
	cv::applyColorMap(dispc, out, cv::COLORMAP_TURBO);
	#else
	cv::applyColorMap(dispc, out, cv::COLORMAP_INFERNO);
	#endif
}

////////////////////////////////////////////////////////////////////////////////

struct MiddleburyData {
	const std::string name;
	const cv::Mat imL;
	const cv::Mat imR;
	const cv::Mat gtL;
	const cv::Mat maskL;
	const MiddEvalCalib calib;
};

struct Result {
	std::vector<MiddEvalResult> results;
	cv::Mat disp;
};

static void run_censussgm(MiddleburyData &data, cv::Mat &disparity) {
	auto stereo = StereoCensusSgm();
	stereo.params.P1 = 4;
	stereo.params.P2 = 18;

	stereo.params.d_min = data.calib.vmin;
	stereo.params.d_max = data.calib.vmax;
	stereo.params.subpixel = 1;
	stereo.params.debug = false;
	stereo.compute(data.imL, data.imR, disparity);
}

static void run_misgm(MiddleburyData &data, cv::Mat &disparity) {
	auto stereo = StereoMiSgm();
	stereo.params.P1 = 4;
	stereo.params.P2 = 18;

	stereo.params.d_min = data.calib.vmin;
	stereo.params.d_max = data.calib.vmax;
	stereo.params.subpixel = 1;
	stereo.params.debug = false;
	stereo.compute(data.imL, data.imR, disparity);
	stereo.setPrior(disparity);
	stereo.compute(data.imL, data.imR, disparity);
}


static void run_gtsgm(MiddleburyData &data, cv::Mat &disparity) {
	auto stereo = StereoGtSgm();
	stereo.params.P1 = 0.1;
	stereo.params.P2 = 1.0;

	stereo.params.d_min = data.calib.vmin;
	stereo.params.d_max = data.calib.vmax;
	stereo.params.subpixel = 1;
	stereo.params.debug = true;
	stereo.setPrior(data.gtL);
	stereo.compute(data.imL, data.imR, disparity);
}

static const std::map<std::string, std::function<void(MiddleburyData&, cv::Mat&)>> algorithms = {
	{ "censussgm", run_censussgm },
	{ "misgm", run_misgm },
};

////////////////////////////////////////////////////////////////////////////////

MiddleburyData load_input(const std::string &path) {
	cv::Mat imL;
	cv::Mat imR;
	cv::Mat gtL;
	cv::Mat maskL;

	imL = cv::imread(path + std::string("im0.png"));
	imR = cv::imread(path + std::string("im1.png"));
	gtL = read_pfm(path + std::string("disp0.pfm"));
	if (gtL.empty()) {
		gtL = read_pfm(path + std::string("disp0GT.pfm"));
	}
	//maskL = cv::imread(argv[1] + std::string("mask0nocc.png"), cv::IMREAD_GRAYSCALE);
	auto calib = read_calibration(path + std::string("calib.txt"));

	maskL.create(imL.size(), CV_8UC1);
	maskL.setTo(cv::Scalar(255));

	if (imL.empty() || imR.empty() || gtL.empty() || maskL.empty()) {
		throw std::exception();
	}

	std::string name;
	name = path.substr(0, path.size()-1);
	name = name.substr(name.find_last_of("/") + 1);

	return {name, imL, imR, gtL, maskL, calib};
}

Result run_one(MiddleburyData &input, std::function<void(MiddleburyData&, cv::Mat&)> f) {
	Result result;
	f(input, result.disp);
	for (const float t : {4.0f, 1.0f, 0.5f, 0.25f}) {
		result.results.push_back(evaluate(result.disp, input.gtL, input.maskL, t));
	}
	return result;
}

Result run_one(MiddleburyData &input, std::string name) {
	return run_one(input, algorithms.at(name));
}

std::map<std::string, Result> run_all(MiddleburyData &input) {
	std::map<std::string, Result> results;

	for (auto const& [name, f] : algorithms) {
		results[name] = run_one(input, f);
	}

	return results;
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat visualize_error(const cv::Mat &disp, const cv::Mat &gt, float t) {
	cv::Mat err(gt.size(), CV_32FC1);
	cv::Mat err_color(gt.size(), CV_8UC1);

	cv::absdiff(gt, disp, err);
	err.setTo(t, err > t);
	err += 1.0f;
	err.setTo(0, disp == 0.0);
	err.convertTo(err, CV_8UC1, 255.0f/(t+1.0f));
	cv::applyColorMap(err, err_color, cv::COLORMAP_PLASMA);

	return err_color;
}

void show_result(std::string name, const MiddleburyData &data, const Result &result) {
	cv::Mat err_color = visualize_error(result.disp, data.gtL, 4.0);
	cv::Mat disp_color(data.gtL.size(), CV_8UC1);

	colorize(result.disp, disp_color, data.calib.ndisp);
	cv::imshow(data.name + " (disparity) - " + name, disp_color);
	cv::imshow(data.name + " (error) - " + name, err_color);

	printf("\n%s: %s\n", name.c_str(), data.name.c_str());
	for (auto res : result.results) {
		printf("%9.2f %%    correct (err < %.2f)\n", 100.0f * res.err_bad_nonoccl, res.threshold);
		printf("%9.2f px   RMSE\n", res.rms_bad_nonoccl);
	}
}

void show_results( const MiddleburyData &data, const std::map<std::string, Result> &results) {
	for (auto const& [name, result] : results) {
		show_result(name, data, result);
	}
}

////////////////////////////////////////////////////////////////////////////////

using cv::Mat;
using std::vector;

int main(int argc, char* argv[]) {
	#if USE_GPU
	std::cout << "GPU VERSION\n";
	#else
	std::cout << "CPU VERSION\n";
	#endif

	if (argc < 2) {
		std::cerr << "usage: path [algorithm]\n";
		return 1;
	}

	if (argc == 2) {
		auto input = load_input(argv[1]);
		auto results = run_all(input);
		show_results(input, results);
		while(cv::waitKey() != 27);
		return 0;
	}
	else if (argc == 3) {
		auto input = load_input(argv[1]);
		auto result = run_one(input, argv[2]);
		show_result(argv[2], input, result);
		while(cv::waitKey() != 27);
		return 0;
	}

	// todo: save results

	std::cerr << "error";
	return 1;
}
