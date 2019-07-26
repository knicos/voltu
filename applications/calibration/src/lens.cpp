#include "common.hpp"
#include "lens.hpp"

#include <ftl/config.h>

#include <loguru.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <vector>

using std::map;
using std::string;
using std::vector;

using cv::Mat;
using cv::Vec2f;
using cv::Vec3f;
using cv::Size;

using namespace ftl::calibration;

void ftl::calibration::intrinsic(map<string, string> &opt) {
	LOG(INFO) << "Begin intrinsic calibration";

	// TODO PARAMETERS TO CONFIG FILE
	const Size image_size = Size(	getOptionInt(opt, "width", 1280),
							getOptionInt(opt, "height", 720));
	const int n_cameras = getOptionInt(opt, "n_cameras", 2);
	const int iter = getOptionInt(opt, "iter", 60);
	const int delay = getOptionInt(opt, "delay", 750);
	const double aperture_width = getOptionDouble(opt, "aperture_width", 6.2);
	const double aperture_height = getOptionDouble(opt, "aperture_height", 4.6);
	const string filename_intrinsics = getOptionString(opt, "profile", FTL_LOCAL_CONFIG_ROOT "/intrinsics.yml");
	CalibrationChessboard calib(opt);

	LOG(INFO) << "Intrinsic calibration parameters";
	LOG(INFO) << "         profile: " << filename_intrinsics;
	LOG(INFO) << "       n_cameras: " << n_cameras;
	LOG(INFO) << "           width: " << image_size.width;
	LOG(INFO) << "          height: " << image_size.height;
	LOG(INFO) << "            iter: " << iter;
	LOG(INFO) << "           delay: " << delay;
	LOG(INFO) << "  aperture_width: " << aperture_width;
	LOG(INFO) << " aperture_height: " << aperture_height;
	LOG(INFO) << "-----------------------------------";

	int calibrate_flags = cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_ASPECT_RATIO;
	// PARAMETERS

	vector<cv::VideoCapture> cameras;
	cameras.reserve(n_cameras);
	for (int c = 0; c < n_cameras; c++) { cameras.emplace_back(c); }
	for (auto &camera : cameras) {
		if (!camera.isOpened()) {
			LOG(ERROR) << "Could not open camera device";
			return;
		}
		camera.set(cv::CAP_PROP_FRAME_WIDTH, image_size.width); 
		camera.set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
	}

	vector<vector<vector<Vec2f>>> image_points(n_cameras);
	vector<vector<vector<Vec3f>>> object_points(n_cameras);
	vector<Mat> img(n_cameras);
	vector<int> count(n_cameras, 0);

	while (iter > *std::min_element(count.begin(), count.end())) {

		for (auto &camera : cameras) { camera.grab(); }

		for (int c = 0; c < n_cameras; c++) {
			vector<Vec2f> points;
			cameras[c].retrieve(img[c]);
		
			if (calib.findPoints(img[c], points)) {
				calib.drawPoints(img[c], points);
				count[c]++;
			}
			else { continue; }
		
			vector<Vec3f> points_ref;
			calib.objectPoints(points_ref);

			Mat camera_matrix, dist_coeffs;
			vector<Mat> rvecs, tvecs;
		
			image_points[c].push_back(points);
			object_points[c].push_back(points_ref);
		}
		
		for (int c = 0; c < n_cameras; c++) {
			cv::imshow("Camera " + std::to_string(c), img[c]);
		}

		cv::waitKey(delay);
	}
	
	vector<Mat> camera_matrix(n_cameras), dist_coeffs(n_cameras);

	for (int c = 0; c < n_cameras; c++) {
		LOG(INFO) << "Calculating intrinsic paramters for camera " << std::to_string(c);
		vector<Mat> rvecs, tvecs;
		
		double rms = cv::calibrateCamera(
							object_points[c], image_points[c],
							image_size, camera_matrix[c], dist_coeffs[c],
							rvecs, tvecs, calibrate_flags
		);

		LOG(INFO) << "final reprojection RMS error: " << rms;

		double fovx, fovy, focal_length, aspect_ratio;
		cv::Point2d principal_point;

		// TODO: check for valid aperture width/height; do not run if not valid values
		cv::calibrationMatrixValues(camera_matrix[c], image_size, aperture_width, aperture_height,
									fovx, fovy, focal_length, principal_point, aspect_ratio);
		
		LOG(INFO) << "";
		LOG(INFO) << "            fovx (deg): " << fovx;
		LOG(INFO) << "            fovy (deg): " << fovy;
		LOG(INFO) << "     focal length (mm): " << focal_length;
		LOG(INFO) << "  principal point (mm): " << principal_point;
		LOG(INFO) << "          aspect ratio: " << aspect_ratio;
		LOG(INFO) << "";
		LOG(INFO) << "Camera matrix:\n" << camera_matrix[c];
		LOG(INFO) << "Distortion coefficients:\n" << dist_coeffs[c];
		LOG(INFO) << "";
	}

	saveIntrinsics(filename_intrinsics, camera_matrix, dist_coeffs);
	LOG(INFO) << "intrinsic paramaters saved to: " << filename_intrinsics;
	
	vector<Mat> map1(n_cameras), map2(n_cameras);
	for (int c = 0; c < n_cameras; c++) {
		cv::initUndistortRectifyMap(camera_matrix[c], dist_coeffs[c], Mat::eye(3,3, CV_64F), camera_matrix[c],
									image_size, CV_16SC2, map1[c], map2[c]);
	}
	
	while (cv::waitKey(25) != 27) {
		for (auto &camera : cameras ) {	camera.grab(); }
		for (int c = 0; c < n_cameras; c++) {
			cameras[c].retrieve(img[c]);
			cv::remap(img[c], img[c], map1[c], map2[c], cv::INTER_CUBIC);
			cv::imshow("Camera " + std::to_string(c), img[c]);
		}
	}
}
