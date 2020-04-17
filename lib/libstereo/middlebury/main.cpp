#include <opencv2/opencv.hpp>

#include <vector>
#include <chrono>

#include "middlebury.hpp"
#include "stereo.hpp"

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

	//cv::applyColorMap(dispc, out, cv::COLORMAP_TURBO);
	cv::applyColorMap(dispc, out, cv::COLORMAP_INFERNO);
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

	Mat imL;
	Mat imR;
	Mat gtL;
	Mat maskL;
	MiddEvalCalib calib;

	if (argc < 2) {
		std::cerr << "usage: middlebury [path]\n";
		return 1;
	}

	imL = cv::imread(argv[1] + std::string("im0.png"));
	imR = cv::imread(argv[1] + std::string("im1.png"));
	gtL = read_pfm(argv[1] + std::string("disp0.pfm"));
	if (gtL.empty()) {
		gtL = read_pfm(argv[1] + std::string("disp0GT.pfm"));
	}
	//maskL = cv::imread(argv[1] + std::string("mask0nocc.png"), cv::IMREAD_GRAYSCALE);
	calib = read_calibration(argv[1] + std::string("calib.txt"));

	maskL.create(imL.size(), CV_8UC1);
	maskL.setTo(cv::Scalar(255));

	int ndisp = calib.vmax - calib.vmin;

	auto stereo = StereoCensusSgm();
	stereo.params.P1 = 7;
	stereo.params.P2 = 60;

	stereo.params.d_min = calib.vmin;
	stereo.params.d_max = calib.vmax;
	stereo.params.subpixel = 1;
	stereo.params.debug = true;
	//stereo.params.paths = AggregationDirections::ALL;
	//stereo.params.uniqueness = 200;

	int i_max = 1;
	float t = 4.0f;

	if (imL.empty() || imR.empty() || gtL.empty()) {
		std::cerr << "can't load image\n";
		return 1;
	}

	if (imL.size() != imR.size()) {
		std::cerr << "images must be same size\n";
		return 1;
	}

	Mat disp(imL.size(), CV_32FC1, cv::Scalar(0.0f));
	//stereo.setPrior(gtL);

	std::cout << "resolution: " << imL.cols << "x" << imL.rows << ", " << ndisp << " disparities [" << calib.vmin << "," << calib.vmax << "]\n";

	std::vector<cv::Mat> disp_color;
	std::vector<cv::Mat> err_color;

	for (int i = 0; i < i_max; i++) {
		auto start = std::chrono::high_resolution_clock::now();

		cv::cuda::GpuMat gpu_imL(imL);
		cv::cuda::GpuMat gpu_imR(imR);
		cv::cuda::GpuMat gpu_disp(disp);
		stereo.compute(gpu_imL, gpu_imR, gpu_disp);
		gpu_disp.download(disp);

		//stereo.compute(imL, imR, disp);
		//stereo.setPrior(disp);
		auto stop = std::chrono::high_resolution_clock::now();
		std::cout	<< "disparity complete in "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(stop-start).count()
					<< " ms\n";

		MiddEvalResult res;

		std::cout << "TYPE: " << gtL.type() << std::endl;

		for (const float t : {4.0f, 1.0f, 0.5f, 0.25f}) {
			res = evaluate(disp, gtL, maskL, t);
			if (i == 0 || i == i_max-1) {
				printf("%9.2f %%    correct (err < %.2f)\n", 100.0f * res.err_bad, res.threshold);
				//printf("%9.2f %%    correct (non-occluded, err < %.2f)\n", 100.0f * res.err_bad_nonoccl, res.threshold);
				printf("%9.2f px   RMSE (all)\n", res.rms_bad);
				//printf("%9.2f px   RMSE (non-occluded)\n", res.rms_bad_nonoccl);
			}
		}

		if (i == 0 || i == i_max-1) {
			printf("%9.2f %%    total\n", 100.0f * res.err_total);
			printf("%9.2f %%    non-occluded\n", 100.0f * res.err_nonoccl);
		}

		Mat &err_color_ = err_color.emplace_back();
		Mat &disp_color_ = disp_color.emplace_back();

		// visualize error: set missing values at 0 and scale error to [1.0, t+1.0]
		// errors > t truncate to t
		Mat err(gtL.size(), CV_32FC1);
		cv::absdiff(gtL, disp, err);
		err.setTo(t, err > t);
		err += 1.0f;
		err.setTo(0, disp == 0.0);
		err.convertTo(err, CV_8UC1, 255.0f/(t+1.0f));
		cv::applyColorMap(err, err_color_, cv::COLORMAP_PLASMA);

		colorize(disp, disp_color_, calib.ndisp);
	}

	cv::imshow(std::to_string(0) + ": " + "error (err < " + std::to_string(t) + ")", err_color[0]);
	cv::imshow(std::to_string(0) + ": " + "disparity", disp_color[0]);
	cv::imshow(std::to_string(i_max-1) + ": " + "error (err < " + std::to_string(t) + ")", err_color[i_max-1]);
	cv::imshow(std::to_string(i_max-1) + ": " + "disparity", disp_color[i_max-1]);

	Mat gt_color;
	colorize(gtL, gt_color, calib.ndisp);
	cv::imshow("gt", gt_color);
	cv::waitKey();

	return 0;
}
