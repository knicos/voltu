#include "common.hpp"
#include "lens.hpp"

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
	Size image_size = Size(	getOptionInt(opt, "width", 1280),
							getOptionInt(opt, "height", 720));
	int iter = getOptionInt(opt, "iter", 60);
	int delay = getOptionInt(opt, "delay", 750);
	string filename_intrinsics = getOptionString(opt, "profile", "./panasonic.yml");
	CalibrationChessboard calib(opt);

	LOG(INFO) << "Intrinsic calibration parameters";
	LOG(INFO) << "     profile: " << filename_intrinsics;
	LOG(INFO) << "       width: " << image_size.width;
	LOG(INFO) << "      height: " << image_size.height;
	LOG(INFO) << "        iter: " << iter;
	LOG(INFO) << "       delay: " << delay;
	LOG(INFO) << "-----------------------------------";

	int calibrate_flags = cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_ASPECT_RATIO;
	// PARAMETERS

	cv::VideoCapture camera = cv::VideoCapture(0);

	if (!camera.isOpened()) {
		LOG(ERROR) << "Could not open camera device";
		return;
	}
	camera.set(cv::CAP_PROP_FRAME_WIDTH, image_size.width); 
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);

	vector<vector<Vec2f>> image_points;
	vector<vector<Vec3f>> object_points;
	
	while (iter > 0) {
		Mat img;
		vector<Vec2f> points;
		
		camera.grab();
		camera.retrieve(img);
		
		bool found = calib.findPoints(img, points);
		if (found) { calib.drawPoints(img, points); }

		cv::imshow("Camera", img);
		cv::waitKey(delay);

		if (!found) { continue; }

		vector<Vec3f> points_ref;
		calib.objectPoints(points_ref);

		Mat camera_matrix, dist_coeffs;
		vector<Mat> rvecs, tvecs;
		
		image_points.push_back(points);
		object_points.push_back(points_ref);

		iter--;
	}
	
	Mat camera_matrix, dist_coeffs;
	vector<Mat> rvecs, tvecs;
	
	double rms = cv::calibrateCamera(
						object_points, image_points,
						image_size, camera_matrix, dist_coeffs,
						rvecs, tvecs, calibrate_flags
	);

	LOG(INFO) << "final reprojection RMS error: " << rms;
		
	saveIntrinsics(filename_intrinsics, camera_matrix, dist_coeffs);
	LOG(INFO) << "intrinsic paramaters saved to: " << filename_intrinsics;
	
	Mat map1, map2;
	cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, Mat::eye(3,3, CV_64F), camera_matrix,
								image_size, CV_16SC2, map1, map2);

	while (cv::waitKey(25) != 27) {
		Mat img, img_out;
		
		camera.grab();
		camera.retrieve(img);
		cv::remap(img, img_out, map1, map2, cv::INTER_CUBIC);

		cv::imshow("Camera", img_out);
	}
}
