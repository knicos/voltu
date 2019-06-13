#include <loguru.hpp>
#include <ftl/config.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "common.hpp"
#include "stereo.hpp"

using std::vector;
using std::map;
using std::string;

using cv::Mat;
using cv::Vec2f, cv::Vec3f;
using cv::Size;

using cv::stereoCalibrate;

using namespace ftl::calibration;

void ftl::calibration::stereo(map<string, string> &opt) {
	LOG(INFO) << "Begin stereo calibration";

	// TODO PARAMETERS TO CONFIG FILE
	Size image_size = Size(1280, 720);
	int iter = 30;
	double max_error = 1.0;
	float alpha = 0;
	string filename_intrinsics = (hasOption(opt, "profile")) ? getOption(opt, "profile") : "./panasonic.yml";
	CalibrationChessboard calib(opt); // TODO paramters hardcoded in constructor
	// PARAMETERS

	int stereocalibrate_flags =
		cv::CALIB_FIX_INTRINSIC | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_ASPECT_RATIO |
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH | cv::CALIB_RATIONAL_MODEL |
		cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5;

	vector<cv::VideoCapture> cameras { cv::VideoCapture(0), cv::VideoCapture(1) };

	for (auto &camera : cameras ) {
		if (!camera.isOpened()) {
			LOG(ERROR) << "Could not open camera device";
			return;
		}
		camera.set(cv::CAP_PROP_FRAME_WIDTH, image_size.width); 
		camera.set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
	}

	vector<vector<vector<Vec2f>>> image_points(2);
	vector<vector<Vec3f>> object_points;
	vector<Mat> dist_coeffs(2);
	vector<Mat> camera_matrices(2);

	// assume identical cameras; load intrinsic parameters
	loadIntrinsics(filename_intrinsics, camera_matrices[0], dist_coeffs[0]);
	loadIntrinsics(filename_intrinsics, camera_matrices[1], dist_coeffs[1]);
	
	Mat R, T, E, F, per_view_errors;

	while (iter > 0) {
		int res = 0;

		vector<Mat> new_img(2);
		vector<vector<Vec2f>> new_points(2);

		for (size_t i = 0; i < 2; i++) {
			auto &camera = cameras[i];
			auto &img = new_img[i];

			camera.grab();
			camera.retrieve(img);
		}

		for (size_t i = 0; i < 2; i++) {
			auto &img = new_img[i];
			auto &points = new_points[i];
			
			if (calib.findPoints(img, points)) {
				calib.drawPoints(img, points);
				res++;
			}

			cv::imshow("Camera: " + std::to_string(i), img);
		}
		cv::waitKey(750);

		if (res != 2) { LOG(WARNING) << "Input not detected on all inputs"; }
		else {
			vector<Vec3f> points_ref;
			calib.objectPoints(points_ref);
			
			// calculate reprojection error with single pair of images
			// reject it if RMS reprojection error too high
			int flags = stereocalibrate_flags;

			double rms = stereoCalibrate(
						vector<vector<Vec3f>> { points_ref }, 
						vector<vector<Vec2f>> { new_points[0] },
						vector<vector<Vec2f>> { new_points[1] },
						camera_matrices[0], dist_coeffs[0],
						camera_matrices[1], dist_coeffs[1],
						image_size, R, T, E, F, per_view_errors,
						flags);
			
			LOG(INFO) << "rms for pattern: " << rms;
			if (rms > max_error) {
				LOG(WARNING) << "RMS reprojection error too high, maximum allowed error: " << max_error;
				continue;
			}

			object_points.push_back(points_ref);
			for (size_t i = 0; i < 2; i++) {
				image_points[i].push_back(new_points[i]);
			}

			iter--;
		}
	}

	// calculate stereoCalibration using all input images (which have low enough
	// RMS error in previous step)

	double rms = stereoCalibrate(object_points,
		image_points[0], image_points[1],
		camera_matrices[0], dist_coeffs[0],
		camera_matrices[1], dist_coeffs[1],
		image_size, R, T, E, F, per_view_errors,
		stereocalibrate_flags,
		cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 120, 1e-6)
	);

	LOG(INFO) << "Final extrinsic calibration RMS (reprojection error): " << rms;
	for (int i = 0; i < per_view_errors.rows * per_view_errors.cols; i++) {
		LOG(4) << "error for sample " << i << ": " << ((double*) per_view_errors.data)[i];
	}

	Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];

	// calculate extrinsic parameters
	stereoRectify(
		camera_matrices[0], dist_coeffs[0],
		camera_matrices[1], dist_coeffs[1],
		image_size, R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, alpha, image_size,
		&validRoi[0], &validRoi[1]
	);

	saveExtrinsics(FTL_LOCAL_CONFIG_ROOT "/extrinsics.yml", R, T, R1, R2, P1, P2, Q);
	LOG(INFO) << "Stereo camera extrinsics saved to: " << FTL_LOCAL_CONFIG_ROOT "/extrinsics.yml";

	// display results
	vector<Mat> map1(2), map2(2);
	cv::initUndistortRectifyMap(camera_matrices[0], dist_coeffs[0], R1, P1, image_size, CV_16SC2, map1[0], map2[0]);
	cv::initUndistortRectifyMap(camera_matrices[1], dist_coeffs[1], R2, P2, image_size, CV_16SC2, map1[1], map2[1]);

	vector<Mat> in(2);
	vector<Mat> out(2);

	while(cv::waitKey(50) == -1) {
		for(size_t i = 0; i < 2; i++) {
			auto &camera = cameras[i];
			camera.grab();
			camera.retrieve(in[i]);
			cv::imshow("Camera: " + std::to_string(i), in[i]);
			cv::remap(in[i], out[i], map1[i], map2[i], cv::INTER_CUBIC);

			// draw lines
			for (int r = 0; r < image_size.height; r = r+50) {
				cv::line(out[i], cv::Point(0, r), cv::Point(image_size.width-1, r), cv::Scalar(0,0,255), 1);
			}

			cv::imshow("Camera " + std::to_string(i) + " (rectified)", out[i]);
		}
	}
}
