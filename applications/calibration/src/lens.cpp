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
#include <atomic>
#include <thread>

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
	const Size image_size = Size(	getOptionInt(opt, "width", 1920),
							getOptionInt(opt, "height", 1080));
	const int n_cameras = getOptionInt(opt, "n_cameras", 2);
	const int iter = getOptionInt(opt, "iter", 40);
	const int delay = getOptionInt(opt, "delay", 1000);
	const double aperture_width = getOptionDouble(opt, "aperture_width", 6.2);
	const double aperture_height = getOptionDouble(opt, "aperture_height", 4.6);
	const string filename_intrinsics = getOptionString(opt, "profile", FTL_LOCAL_CONFIG_ROOT "/calibration.yml");
	CalibrationChessboard calib(opt);
	bool use_guess = getOptionInt(opt, "use_guess", 1);
	//bool use_guess_distortion = getOptionInt(opt, "use_guess_distortion", 0);

	LOG(INFO) << "Intrinsic calibration parameters";
	LOG(INFO) << "               profile: " << filename_intrinsics;
	LOG(INFO) << "             n_cameras: " << n_cameras;
	LOG(INFO) << "                 width: " << image_size.width;
	LOG(INFO) << "                height: " << image_size.height;
	LOG(INFO) << "                  iter: " << iter;
	LOG(INFO) << "                 delay: " << delay;
	LOG(INFO) << "        aperture_width: " << aperture_width;
	LOG(INFO) << "       aperture_height: " << aperture_height;
	LOG(INFO) << "             use_guess: " << use_guess << "\n";
	LOG(WARNING) << "WARNING: This application overwrites existing files and does not previous values!";
	//LOG(INFO) << "  use_guess_distortion: " << use_guess_distortion;

	LOG(INFO) << "-----------------------------------";

	int calibrate_flags =	cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_ASPECT_RATIO;
	if (use_guess) { calibrate_flags |= cv::CALIB_USE_INTRINSIC_GUESS; }
	//						cv::CALIB_FIX_PRINCIPAL_POINT;
	// PARAMETERS


	vector<Mat> camera_matrix(n_cameras), dist_coeffs(n_cameras);

	for (Mat &d : dist_coeffs)
	{
		d = Mat(Size(5, 1), CV_64FC1, cv::Scalar(0.0));
	}

	if (use_guess)
	{
		camera_matrix.clear();
		vector<Mat> tmp;
		Size tmp_size;
		
		loadIntrinsics(filename_intrinsics, camera_matrix, tmp, tmp_size);
		CHECK(camera_matrix.size() == n_cameras); // (camera_matrix.size() == dist_coeffs.size())
		if ((tmp_size != image_size) && (!tmp_size.empty()))
		{
			Mat scale = Mat::eye(Size(3, 3), CV_64FC1);
			scale.at<double>(0, 0) = ((double) image_size.width) / ((double) tmp_size.width);
			scale.at<double>(1, 1) = ((double) image_size.height) / ((double) tmp_size.height);
			for (Mat &K : camera_matrix) { K = scale * K; }
		}

		if (tmp_size.empty())
		{
			use_guess = false;
			LOG(FATAL) << "No valid calibration found.";
		}
	}

	vector<cv::VideoCapture> cameras;
	cameras.reserve(n_cameras);
	for (int c = 0; c < n_cameras; c++) { cameras.emplace_back(c); }
	for (auto &camera : cameras)
	{
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
	vector<Mat> img_display(n_cameras);
	vector<int> count(n_cameras, 0);
	Mat display(Size(image_size.width * n_cameras, image_size.height), CV_8UC3);

	for (int c = 0; c < n_cameras; c++)
	{
		img_display[c] = Mat(display, cv::Rect(c * image_size.width, 0, image_size.width, image_size.height));
	}

	std::mutex m;
	std::atomic<bool> ready = false;
	auto capture = std::thread([n_cameras, delay, &m, &ready, &count, &calib, &img, &image_points, &object_points]()
	{
		vector<Mat> tmp(n_cameras);
		while(true)
		{
			if (!ready)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(delay));
				continue;
			}

			m.lock();
			ready = false;
			for (int c = 0; c < n_cameras; c++)
			{
				img[c].copyTo(tmp[c]);
			}
			m.unlock();
			
			for (int c = 0; c < n_cameras; c++)
			{
				vector<Vec2f> points;
				if (calib.findPoints(tmp[c], points))
				{
					count[c]++;
				}
				else { continue; }

				vector<Vec3f> points_ref;
				calib.objectPoints(points_ref);
				Mat camera_matrix, dist_coeffs;
				image_points[c].push_back(points);
				object_points[c].push_back(points_ref);
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(delay));
		}
	});

	while (iter > *std::min_element(count.begin(), count.end()))
	{
		if (m.try_lock())
		{
			for (auto &camera : cameras) { camera.grab(); }

			for (int c = 0; c < n_cameras; c++)
			{
				cameras[c].retrieve(img[c]);
			}

			ready = true;
			m.unlock();
		}
		
		for (int c = 0; c < n_cameras; c++)
		{
			img[c].copyTo(img_display[c]);
			m.lock();

			if (image_points[c].size() > 0)
			{
				
				for (auto &points : image_points[c])
				{
					calib.drawCorners(img_display[c], points);
				}

				calib.drawPoints(img_display[c], image_points[c].back());
			}

			m.unlock();
		}

		cv::namedWindow("Cameras", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Cameras", display);

		cv::waitKey(10);
	}

	cv::destroyAllWindows();

	for (int c = 0; c < n_cameras; c++)
	{
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

	saveIntrinsics(filename_intrinsics, camera_matrix, dist_coeffs, image_size);
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
