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
	// image size, also used by CalibrationChessboard
	Size image_size = Size(	getOptionInt(opt, "width", 1280),
							getOptionInt(opt, "height", 720));
	// iterations
	int iter = getOptionInt(opt, "iter", 50);
	// delay between images
	double delay = getOptionInt(opt, "delay", 250);
	// max_error for a single image; if error larger image discarded
	double max_error = getOptionDouble(opt, "max_error", 1.0);
	// scaling/cropping (see OpenCV stereoRectify())
	float alpha = getOptionDouble(opt, "alpha", 0);
	// intrinsics filename
	string filename_intrinsics = getOptionString(opt, "profile", "./panasonic.yml");

	bool use_grid = (bool) getOptionInt(opt, "use_grid", 0);

	LOG(INFO) << "Stereo calibration parameters";
	LOG(INFO) << "     profile: " << filename_intrinsics;
	LOG(INFO) << "       width: " << image_size.width;
	LOG(INFO) << "      height: " << image_size.height;
	LOG(INFO) << "        iter: " << iter;
	LOG(INFO) << "       delay: " << delay;
	LOG(INFO) << "   max_error: " << max_error;
	LOG(INFO) << "       alpha: " << alpha;
	LOG(INFO) << "    use_grid: " << use_grid;
	LOG(INFO) << "-----------------------------------";

	CalibrationChessboard calib(opt);
	vector<Grid> grids;
	int grid_i = 0;

	// grid parameters, 3x3 grid; one small grid and one large grid. Grids are cycled until
	// iter reaches zero
	grids.push_back(Grid(3, 3,
						(3.0f/4.0f) * image_size.width, (3.0f/4.0f) * image_size.height,
						((1.0f/4.0f) * image_size.width) / 2, ((1.0f/4.0f) * image_size.height) / 2));

	grids.push_back(Grid(3, 3, image_size.width, image_size.height, 0, 0));
	Grid grid = grids[grid_i];

	// PARAMETERS

	int stereocalibrate_flags =
		cv::CALIB_FIX_INTRINSIC | cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_ASPECT_RATIO |
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH | 
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

	// image points to calculate the parameters after all input data is captured
	vector<vector<vector<Vec2f>>> image_points(2);
	vector<vector<Vec3f>> object_points;

	// image points for each grid, updated to image_points and object_points
	// after is grid complete
	vector<vector<vector<Vec2f>>> image_points_grid(9, vector<vector<Vec2f>>(2));
	vector<vector<Vec3f>> object_points_grid(9);
	
	vector<Mat> dist_coeffs(2);
	vector<Mat> camera_matrices(2);
	Size intrinsic_resolution;
	if (!loadIntrinsics(filename_intrinsics, camera_matrices, dist_coeffs, intrinsic_resolution))
	{
		LOG(FATAL) << "Failed to load intrinsic camera parameters from file.";
	}
	
	if (intrinsic_resolution != image_size)
	{
		LOG(FATAL) << "Intrinsic resolution is not same as input resolution (TODO)";
	}

	Mat R, T, E, F, per_view_errors;
	
	// capture calibration patterns
	while (iter > 0) {
		int res = 0;
		int grid_pos = -1;

		vector<Mat> new_img(2);
		vector<vector<Vec2f>> new_points(2);

		int delay_remaining = delay;
		for (; delay_remaining > 50; delay_remaining -= 50) {
			cv::waitKey(50);

			for (size_t i = 0; i < 2; i++) {
				auto &camera = cameras[i];
				auto &img = new_img[i];

				camera.grab();
				camera.retrieve(img);

				if (use_grid && i == 0) grid.drawGrid(img);
				cv::imshow("Camera " + std::to_string(i), img);
			}
		}

		for (size_t i = 0; i < 2; i++) {
			auto &img = new_img[i];
			auto &points = new_points[i];

			// TODO move to "findPoints"-thread
			if (calib.findPoints(img, points)) {
				calib.drawPoints(img, points);
				res++;
			}

			cv::imshow("Camera " + std::to_string(i), img);
		}

		if (res != 2) { LOG(WARNING) << "Input not detected on all inputs"; continue; }
		
		if (use_grid) {
			// top left and bottom right corners; not perfect but good enough
			grid_pos = grid.checkGrid(
				cv::Point(new_points[0][0]),
				cv::Point(new_points[0][new_points[0].size()-1])
			);

			if (grid_pos == -1) { LOG(WARNING) << "Captured pattern not inside grid cell"; continue; }
		}

		vector<Vec3f> points_ref;
		calib.objectPoints(points_ref);
		
		/* doesn't seem to be very helpful (error almost always low enough)
		// calculate reprojection error with single pair of images
		// reject it if RMS reprojection error too high
		int flags = stereocalibrate_flags;
		
		double rms_iter = stereoCalibrate(
					vector<vector<Vec3f>> { points_ref }, 
					vector<vector<Vec2f>> { new_points[0] },
					vector<vector<Vec2f>> { new_points[1] },
					camera_matrices[0], dist_coeffs[0],
					camera_matrices[1], dist_coeffs[1],
					image_size, R, T, E, F, per_view_errors,
					flags);
		
		LOG(INFO) << "rms for pattern: " << rms_iter;
		if (rms_iter > max_error) {
			LOG(WARNING) << "RMS reprojection error too high, maximum allowed error: " << max_error;
			continue;
		}*/
		
		if (use_grid) {
			// store results in result grid
			object_points_grid[grid_pos] = points_ref;
			for (size_t i = 0; i < 2; i++) { image_points_grid[grid_pos][i] = new_points[i]; }
			
			grid.updateGrid(grid_pos);

			if (grid.isComplete()) {
				LOG(INFO) << "Grid complete";
				grid.reset();
				grid_i = (grid_i + 1) % grids.size();
				grid = grids[grid_i];

				// copy results
				object_points.insert(object_points.end(), object_points_grid.begin(), object_points_grid.end());
				for (size_t i = 0; i < image_points_grid.size(); i++) {
					for (size_t j = 0; j < 2; j++) { image_points[j].push_back(image_points_grid[i][j]); }
				}
				iter--;
			}
		}
		else {
			object_points.push_back(points_ref);
			for (size_t i = 0; i < 2; i++) { image_points[i].push_back(new_points[i]); }
			iter--;
		}
	}

	// calculate stereoCalibration using all input images (which have low enough
	// RMS error in previous step)

	LOG(INFO) << "Calculating extrinsic stereo parameters using " << object_points.size() << " samples.";

	CHECK(object_points.size() == image_points[0].size());
	CHECK(object_points.size() == image_points[1].size());

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

	stereoRectify(
		camera_matrices[0], dist_coeffs[0],
		camera_matrices[1], dist_coeffs[1],
		image_size, R, T, R1, R2, P1, P2, Q,
		0, alpha, image_size,
		&validRoi[0], &validRoi[1]
	);

	saveExtrinsics(FTL_LOCAL_CONFIG_ROOT "/extrinsics.yml", R, T, R1, R2, P1, P2, Q);
	LOG(INFO) << "Stereo camera extrinsics saved to: " << FTL_LOCAL_CONFIG_ROOT "/extrinsics.yml";

	for (size_t i = 0; i < 2; i++) { cv::destroyWindow("Camera " + std::to_string(i)); }

	// Visualize results
	vector<Mat> map1(2), map2(2);
	cv::initUndistortRectifyMap(camera_matrices[0], dist_coeffs[0], R1, P1, image_size, CV_16SC2, map1[0], map2[0]);
	cv::initUndistortRectifyMap(camera_matrices[1], dist_coeffs[1], R2, P2, image_size, CV_16SC2, map1[1], map2[1]);

	vector<Mat> in(2);
	vector<Mat> out(2);
	// vector<Mat> out_gray(2);
	// Mat diff, diff_color;

	while(cv::waitKey(25) == -1) {
		for(size_t i = 0; i < 2; i++) {
			auto &camera = cameras[i];
			camera.grab();
			camera.retrieve(in[i]);
	
			auto p = cv::Point2i(camera_matrices[i].at<double>(0, 2), camera_matrices[i].at<double>(1, 2));
			cv::drawMarker(in[i], p, cv::Scalar(51, 204, 51), cv::MARKER_CROSS, 40, 1);
			cv::drawMarker(in[i], p, cv::Scalar(51, 204, 51), cv::MARKER_SQUARE, 25);

			cv::remap(in[i], out[i], map1[i], map2[i], cv::INTER_CUBIC);

			// draw lines
			for (int r = 50; r < image_size.height; r = r+50) {
				cv::line(out[i], cv::Point(0, r), cv::Point(image_size.width-1, r), cv::Scalar(0,0,255), 1);
			}

			if (i == 0) { // left camera
				auto p_r = cv::Point2i(-Q.at<double>(0, 3), -Q.at<double>(1, 3));
				cv::drawMarker(out[i], p_r, cv::Scalar(0, 0, 204), cv::MARKER_CROSS, 30);
				cv::drawMarker(out[i], p_r, cv::Scalar(0, 0, 204), cv::MARKER_SQUARE);
			}
			
			cv::imshow("Camera " + std::to_string(i) + " (unrectified)", in[i]);
			cv::imshow("Camera " + std::to_string(i) + " (rectified)", out[i]);
		}
		
		/* not useful
		cv::absdiff(out_gray[0], out_gray[1], diff);
		cv::applyColorMap(diff, diff_color, cv::COLORMAP_JET);
		cv::imshow("Difference", diff_color);
		*/
	}

	cv::destroyAllWindows();
}
