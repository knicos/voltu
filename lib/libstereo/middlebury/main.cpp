#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <utility>
#include <chrono>

#include "middlebury.hpp"
#include "algorithms.hpp"

#include <cuda_runtime.h>

#include "../../components/common/cpp/include/ftl/config.h"

struct Result {
	std::vector<MiddEvalResult> results;
	cv::Mat disp;
};

Result run_one(MiddleburyData &input, Algorithm *stereo) {
	Result result;
	stereo->run(input, result.disp);
	for (const float t : {4.0f, 2.0f, 1.0f, 0.5f}) {
		result.results.push_back(evaluate(result.disp, input.gtL, input.maskL, t));
	}
	return result;
}

Result run_one(MiddleburyData &input, std::string name) {
	return run_one(input, algorithms.at(name));
}

std::map<std::string, Result> run_all(MiddleburyData &input) {
	std::map<std::string, Result> results;

	for (auto &[name, f] : algorithms) {
		results[name] = run_one(input, f);
	}

	return results;
}

////////////////////////////////////////////////////////////////////////////////

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

cv::Mat visualize_error(const cv::Mat &disp, const cv::Mat &gt, float t) {
	cv::Mat err(gt.size(), CV_32FC1);
	cv::Mat err_color(gt.size(), CV_8UC1);
	float bad_t = 0.75;

	cv::absdiff(gt, disp, err);
	err.setTo(t, err > t);
	err += bad_t;
	err.setTo(0, disp == 0.0);
	err.convertTo(err, CV_8UC1, 255.0f/(t+bad_t));
	cv::applyColorMap(err, err_color, cv::COLORMAP_PLASMA);

	return err_color;
}

void show_result(std::string name, const MiddleburyData &data, const Result &result) {
	cv::Mat err_color = visualize_error(result.disp, data.gtL, 4.0);
	cv::Mat disp_color(data.gtL.size(), CV_8UC1);

	colorize(result.disp, disp_color, data.calib.ndisp);
	cv::Mat tmp;
	float scale = 1280.0f / float(disp_color.cols);
	cv::resize(disp_color, tmp, cv::Size(disp_color.cols*scale, disp_color.rows*scale));
	cv::imshow(data.name + " (disparity) - " + name, tmp);
	cv::resize(err_color, tmp, cv::Size(err_color.cols*scale, err_color.rows*scale));
	cv::imshow(data.name + " (error) - " + name, tmp);

	printf("\n%s: %s\n", name.c_str(), data.name.c_str());
	int nt = result.results.size();

	printf("%8s", "");
	for (auto res : result.results) { printf("%10.2f ", res.threshold); }
	printf("\n");

	for (auto res : result.results) { printf("%13s", "-------------"); }
	printf("\n");

	printf("%-8s", "total");
	for (auto res : result.results) { printf("%9.2f%% ", res.err_total*100.0f); }
	printf("\n");

	printf("%-8s", "bad");
	for (auto res : result.results) { printf("%9.2f%% ", res.err_bad*100.0f); }
	printf("\n");

	printf("%-8s", "invalid");
	for (auto res : result.results) { printf("%9.2f%% ", res.err_invalid*100.0f); }
	printf("\n");

	printf("%-8s", "RMSE");
	for (auto res : result.results) { printf("%8.2fpx ", sqrtf(res.mse_good)); }
	printf("\n");

	printf("%s: %2.2fpx, %s: %2.2fpx\n",	"avgerr", result.results[0].avgerr,
											"RMSE (total)", sqrtf(result.results[0].mse_total));
}

void save_result(std::string name, const MiddleburyData &data, const Result &result) {
	cv::Mat err_color = visualize_error(result.disp, data.gtL, 4.0);
	cv::Mat disp_color(data.gtL.size(), CV_8UC1);
	colorize(result.disp, disp_color, data.calib.ndisp);
	cv::imwrite(data.name + "_" + name + "_disp.png", disp_color);
	cv::imwrite(data.name + "_" + name + "_err.png", err_color);
}

////////////////////////////////////////////////////////////////////////////////

/**
 * Calculate disparities and errors for all Middlebury pairs in paths using
 * Algorithms in algorithms (if not specified, uses all available algorithms).
 * Results visualized and error values printed to stdout. Visualization can be
 * saved.
 */
void main_default(const std::vector<std::string> &paths,
		const std::vector<std::string> &algorithms) {

	std::vector<std::pair<MiddleburyData, std::map<std::string, Result>>> results;

	int devices=0;
	cudaGetDeviceCount(&devices);
	cudaSetDevice(devices-1);

	for (const auto &dir : paths) {
		auto input = load_input(dir);
		std::map<std::string, Result> result;

		if (algorithms.size() == 0) {
			result = run_all(input);
		}
		else {
			for (const auto &algorithm : algorithms) {
				result[algorithm] = run_one(input, algorithm);
			}
		}

		results.push_back(std::make_pair(input, result));
	}

	for (const auto &[input, data] : results) {
		for (const auto &[name, result] : data) {
			show_result(name, input, result);
		}
	}

	printf("\nPress S to save (visualization to working directory), ESC to exit\n");

	while(uchar k = cv::waitKey()) {
		if (k == 's') {
			for (const auto &[input, data] : results) {
				for (const auto &[name, result] : data) {
					save_result(name, input, result);
				}
			}
			std::cout << "Saved\n";
		}
		else if (k == 27 || k == 255) {
			return;
		}
	}
}

/**
 * Perform grid search for P1 and P2 with P2 >= P1 minimizing total (R)MSE for
 * whole dataset. Should be used with masked ground truth.
 * (occluded areas not useful)
 */
void main_gridsearch_p1p2(const std::vector<std::string> &paths,
		const std::vector<std::string> &algorithms_str,
		float P1_min=0, float P1_max=128, float P2_max=256, float step=1.0) {

	// Expects path to dataset directory. TODO: If empty, try loading paths
	// separately (assume each path points to directory containing one dataset
	// image pair)
	auto dataset = load_directory(paths.at(0));

	for (auto s : algorithms_str) {
		Algorithm* stereo = algorithms.at(s);
		// density aftel l/r cheeck will affect MSE, mask should be used to
		// ignore occlusions
		stereo->lr_consistency = false;
		float err_min = INFINITY;
		float best_P1 = 0.0f;
		float best_P2 = 0.0f;

		// first axis: P1, second axis P2
		std::vector<std::vector<float>> search_results(P1_max/step);
		for (auto &row : search_results) {
			row = std::vector<float>(P2_max/step, 0.0f);
		}

		for (float P1 = P1_min; P1 < P1_max; P1 += step) {
			for (float P2 = P1; P2 < P2_max; P2 += step) { // constraint: P2 >= P1

				stereo->P1 = P1;
				stereo->P2 = P2;
				float mse = 0.0;
				float n = 0.0;

				for (auto &input : dataset) {
					auto res = run_one(input, stereo);
					auto &res2 = res.results[2];

					float n_update = res2.n - res2.invalid;
					n += n_update;
					mse += res2.mse_total*n_update;
				}
				mse /= n;
				search_results.at((P1 - P1_min)/step)
							  .at((P2 - P1_min)/step) = mse;

				if (mse < err_min) {
					//printf("RMSE: %.4f (P1: %.0f, P2: %.0f)\n", sqrtf(mse), P1, P2);
					err_min = mse;
					best_P1 = P1;
					best_P2 = P2;
				}
			}
		}

		printf("Best paramters for %s: P1=%.0f, P2=%0.f\n", s.c_str(), best_P1, best_P2);
		// TODO: save full results in csv (heatmap plot?)
	}
}

int main(int argc, char* argv[]) {
	#if USE_GPU
	std::cout << "GPU VERSION\n";
	#else
	std::cout << "CPU VERSION\n";
	#endif

	std::vector<std::string> paths;
	std::vector<std::string> algorithms;
	char test_type = 'd';
	int opt;

	while ((opt = getopt(argc, argv, "d:t:a:")) != -1) {
		switch (opt) {
			case 'a':
				algorithms.push_back(optarg);
				break;

			case 'd':
				paths.push_back(optarg);
				break;

			case 't':
				test_type = optarg[0];
				break;

			default: /* '?' */
				printf("Usage: %s [-d directory] [-t test type]\n", argv[0]);
				exit(EXIT_FAILURE);
		}
	}

	if (paths.size() == 0) {
		printf("At least one directory required.");
		exit(EXIT_FAILURE);
	}

	switch (test_type) {
		case 'd':
			/* Default: calculate disparities with given algorithms (or all if
			 * not specified) and visualize results */
			main_default(paths, algorithms);
			exit(EXIT_SUCCESS);
			break;

		case 'g':
			/* grid search P1 and P2 for given algorithms
			 *(parse additional options using getops again?) */
			main_gridsearch_p1p2(paths, algorithms);
			exit(EXIT_SUCCESS);
			break;

		default:
			printf("Unknown test type %c", test_type);
			exit(EXIT_FAILURE);
	}

	return 0;
}
