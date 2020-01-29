#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/group.hpp>

#include <ftl/master.hpp>
#include <ftl/streams/receiver.hpp>
#include <ftl/streams/netstream.hpp>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <numeric>
#include <fstream>

#include "util.hpp"
#include "multicalibrate.hpp"

using std::string;
using std::optional;

using std::list;
using std::vector;
using std::map;
using std::pair;
using std::make_pair;

using cv::Mat;
using cv::Scalar;
using cv::Size;
using cv::Point2f;
using cv::Point2d;
using cv::Point3f;
using cv::Point3d;
using cv::Vec4f;
using cv::Vec4d;

using ftl::net::Universe;
using ftl::rgbd::Source;
using ftl::codecs::Channel;

Mat createCameraMatrix(const ftl::rgbd::Camera &parameters) {
	Mat m = (cv::Mat_<double>(3,3) <<
				parameters.fx,	0.0,			-parameters.cx,
				0.0, 			parameters.fy,	-parameters.cy,
				0.0,			0.0,			 1.0);
	return m;
}

struct CalibrationParams {
	string output_path;
	string registration_file;
	vector<size_t> idx_cameras;
	bool save_extrinsic = true;
	bool save_intrinsic = false;
	bool optimize_intrinsic = false;
	int reference_camera = -1;
	double alpha = 0.0;
	Size size;
};

////////////////////////////////////////////////////////////////////////////////
// Visualization
////////////////////////////////////////////////////////////////////////////////

void stack(const vector<Mat> &img, Mat &out, const int rows, const int cols) {
	Size size = img[0].size();
	Size size_out = Size(size.width * cols, size.height * rows);
	if (size_out != out.size() || out.type() != CV_8UC3) {
		out = Mat(size_out, CV_8UC3, Scalar(0, 0, 0));
	}

	for (size_t i = 0; i < img.size(); i++) {
		int row = i % rows;
		int col = i / rows;
		auto rect = cv::Rect(size.width * col, size.height * row, size.width, size.height);
		img[i].copyTo(out(rect));
	}
}

void stack(const vector<Mat> &img, Mat &out) {
	// TODO
	int rows = 2;
	int cols = (img.size() + 1) / 2;
	stack(img, out, rows, cols);
}

void visualizeCalibration(	MultiCameraCalibrationNew &calib, Mat &out,
				 			vector<Mat> &rgb, const vector<Mat> &map1,
							const vector<Mat> &map2, const vector<cv::Rect> &roi)
{
	vector<Scalar> colors = {
		Scalar(64, 64, 255),
		Scalar(64, 64, 255),
		Scalar(64, 255, 64),
		Scalar(64, 255, 64),
	};
	
	vector<int> markers = {cv::MARKER_SQUARE, cv::MARKER_DIAMOND};

	for (size_t c = 0; c < rgb.size(); c++) {
		cv::remap(rgb[c], rgb[c], map1[c], map2[c], cv::INTER_CUBIC);
		cv::rectangle(rgb[c], roi[c], Scalar(24, 224, 24), 2);

		for (int r = 50; r < rgb[c].rows; r = r+50) {
			cv::line(rgb[c], cv::Point(0, r), cv::Point(rgb[c].cols-1, r), cv::Scalar(0,0,255), 1);
		}

		cv::putText(rgb[c],
			"Camera " + std::to_string(c),
			Point2i(roi[c].x + 10, roi[c].y + 30),
			cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(64, 64, 255), 1);
	}
	stack(rgb, out);
}

////////////////////////////////////////////////////////////////////////////////
// RPC
////////////////////////////////////////////////////////////////////////////////
// Using Mat directly

vector<Mat> getDistortionParametersRPC(ftl::net::Universe* net, ftl::stream::Net* nstream) {
	return net->call<vector<Mat>>(nstream->getPeer(), "get_distortion");
}

bool setRectifyRPC(	ftl::net::Universe* net, ftl::stream::Net* nstream,
					bool enabled) {
	return net->call<bool>(nstream->getPeer(), "set_rectify", enabled);
}

bool setIntrinsicsRPC(	ftl::net::Universe* net, ftl::stream::Net* nstream,
						const Size &size, const vector<Mat> &K, const vector<Mat> &D) {

	return net->call<bool>(nstream->getPeer(), "set_intrinsics",
							size, K[0], D[0], K[1], D[1] );
}

bool setExtrinsicsRPC(	ftl::net::Universe* net, ftl::stream::Net* nstream,
						const Mat &R, const Mat &t) {
	return net->call<bool>(nstream->getPeer(), "set_extrinsics", R, t);
}

bool setPoseRPC(ftl::net::Universe* net, ftl::stream::Net* nstream,
				const Mat &pose) {
	return net->call<bool>(nstream->getPeer(), "set_pose",  pose);
}

bool saveCalibrationRPC(ftl::net::Universe* net, ftl::stream::Net* nstream) {
	return net->call<bool>(nstream->getPeer(), "save_calibration");
}

////////////////////////////////////////////////////////////////////////////////

/* run calibration and perform RPC to update calibration on nodes */

void calibrateRPC(	ftl::net::Universe* net,
					MultiCameraCalibrationNew &calib,
					const CalibrationParams &params,
					vector<ftl::stream::Net*> &nstreams,
					vector<Mat> &map1,
					vector<Mat> &map2,
					vector<cv::Rect> &roi) {
	int reference_camera = params.reference_camera;
	if (params.reference_camera < 0) {
		reference_camera = calib.getOptimalReferenceCamera();
		reference_camera -= (reference_camera & 1);
		LOG(INFO) << "optimal camera (automatic): " << reference_camera;
	}
	LOG(INFO) << "reference camera: " << reference_camera;

	if (params.optimize_intrinsic) calib.setFixIntrinsic(0);

	calib.calibrateAll(reference_camera);
	vector<Mat> R, t;
	calib.getCalibration(R, t);

	size_t n_cameras = calib.getCamerasCount();

	vector<Mat> R_rect(n_cameras), t_rect(n_cameras);
	vector<Mat> Rt_out(n_cameras);
	map1.resize(n_cameras);
	map2.resize(n_cameras);
	roi.resize(n_cameras);

	for (size_t c = 0; c < n_cameras; c += 2) {
		Mat K1 = calib.getCameraMat(c);
		Mat K2 = calib.getCameraMat(c + 1);
		Mat D1 = calib.getDistCoeffs(c);
		Mat D2 = calib.getDistCoeffs(c + 1);
		Mat P1, P2, Q;
		Mat R1, R2;
		Mat R_c1c2, T_c1c2;
		
		calculateTransform(R[c], t[c], R[c + 1], t[c + 1], R_c1c2, T_c1c2);
		cv::stereoRectify(K1, D1, K2, D2, params.size, R_c1c2, T_c1c2, R1, R2, P1, P2, Q, 0, params.alpha);

		R_c1c2 = R_c1c2.clone();
		T_c1c2 = T_c1c2.clone();

		// calculate extrinsics from rectified parameters
		Mat _t = Mat(Size(1, 3), CV_64FC1, Scalar(0.0));
		Rt_out[c] = getMat4x4(R[c], t[c]) * getMat4x4(R1, _t).inv();
		Rt_out[c + 1] = getMat4x4(R[c + 1], t[c + 1]) * getMat4x4(R2, _t).inv();

		LOG(INFO) << K1;
		LOG(INFO) << K2;
		LOG(INFO) << R_c1c2;
		LOG(INFO) << T_c1c2;

		LOG(INFO) << "--------------------------------------------------------";

		auto *nstream = nstreams[c/2];
		while(true) {
			try {
				if (params.optimize_intrinsic) {
					setIntrinsicsRPC(net, nstream, params.size, {K1, K2}, {D1, D2});
				}
				setExtrinsicsRPC(net, nstream, R_c1c2, T_c1c2);
				setPoseRPC(net, nstream, Rt_out[c]);
				saveCalibrationRPC(net, nstream);
				LOG(INFO) << "CALIBRATION SENT";
				break;

			} catch (std::exception &ex) {
				LOG(ERROR) << "RPC failed: " << ex.what();
				sleep(1);
			}
		}

		// for visualization
		Size new_size;
		cv::stereoRectify(K1, D1, K2, D2, params.size, R_c1c2, T_c1c2, R1, R2, P1, P2, Q, 0, 1.0, new_size, &roi[c], &roi[c + 1]);
		//roi[c] = cv::Rect(0, 0, params.size.width, params.size.height);
		//roi[c+1] = cv::Rect(0, 0, params.size.width, params.size.height);
		cv::initUndistortRectifyMap(K1, D1, R1, P1, params.size, CV_16SC2, map1[c], map2[c]);
		cv::initUndistortRectifyMap(K2, D2, R2, P2, params.size, CV_16SC2, map1[c + 1], map2[c + 1]);
	}
}

void runCameraCalibration(	ftl::Configurable* root,
							int n_views, int min_visible,
							string path, string filename,
							bool save_input,
							CalibrationParams &params)
{
	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Master ctrl(root, net);

	net->start();
	net->waitConnections();
	
	ftl::stream::Muxer *stream = ftl::create<ftl::stream::Muxer>(root, "muxstream");
	ftl::stream::Receiver *gen = ftl::create<ftl::stream::Receiver>(root, "receiver");
	gen->setStream(stream);
	auto stream_uris = net->findAll<std::string>("list_streams");
	std::sort(stream_uris.begin(), stream_uris.end());
	std::vector<ftl::stream::Net*> nstreams;

	int count = 0;
	for (auto &s : stream_uris) {
		LOG(INFO) << " --- found stream: " << s;
		auto *nstream = ftl::create<ftl::stream::Net>(stream, std::to_string(count), net);
		std::string name = *(nstream->get<std::string>("name"));
		nstream->set("uri", s);
		nstreams.push_back(nstream);
		stream->add(nstream);
		
		++count;
	}
	
	const size_t n_sources = nstreams.size();
	const size_t n_cameras = n_sources * 2;
	size_t reference_camera = 0;

	std::mutex mutex;
	std::atomic<bool> new_frames = false;
	vector<Mat> rgb_(n_cameras), rgb_new(n_cameras);
	vector<Mat> camera_parameters(n_cameras);
	Size res;

	gen->onFrameSet([stream, &mutex, &new_frames, &rgb_new, &camera_parameters, &res](ftl::rgbd::FrameSet &fs) {
		stream->select(fs.id, Channel::Left + Channel::Right);
		if (fs.frames.size() != (rgb_new.size()/2)) {
			// nstreams.size() == (rgb_new.size()/2)
			LOG(ERROR)	<< "frames.size() != nstreams.size(), "
						<< fs.frames.size() << " != " << (rgb_new.size()/2); 
		}

		UNIQUE_LOCK(mutex, CALLBACK);
		bool good = true;
		try {
			for (size_t i = 0; i < fs.frames.size(); i ++) {
				if (!fs.frames[i].hasChannel(Channel::Left)) {
					good = false;
					LOG(ERROR) << "No left channel";
					break;
				}

				if (!fs.frames[i].hasChannel(Channel::Right)) {
					good = false;
					LOG(ERROR) << "No right channel";
					break;
				}

				auto idx = stream->originStream(0, i);
				CHECK(idx >= 0) << "negative index";
				
				fs.frames[i].download(Channel::Left+Channel::Right);
				Mat &left = fs.frames[i].get<Mat>(Channel::Left);
				Mat &right = fs.frames[i].get<Mat>(Channel::Right);
				
				/*
				// note: also returns empty sometimes 
				fs.frames[i].upload(Channel::Left+Channel::Right);
				Mat left, right;
				fs.frames[i].get<cv::cuda::GpuMat>(Channel::Left).download(left);
				fs.frames[i].get<cv::cuda::GpuMat>(Channel::Right).download(right);
				*/
				
				CHECK(!left.empty() && !right.empty());

				cv::cvtColor(left, rgb_new[2*idx], cv::COLOR_BGRA2BGR);
				cv::cvtColor(right, rgb_new[2*idx+1], cv::COLOR_BGRA2BGR);
				
				camera_parameters[2*idx] = createCameraMatrix(fs.frames[i].getLeftCamera());
				camera_parameters[2*idx+1] = createCameraMatrix(fs.frames[i].getRightCamera());
				if (res.empty()) res = rgb_new[2*idx].size();
			}
		}
		catch (std::exception ex) {
			LOG(ERROR) << "exception: " << ex.what();
			good = false;
		}
		catch (...)  {
			LOG(ERROR) << "unknown exception";
			good = false;
		}
		new_frames = good;
		return true;
	});

	stream->begin();
	ftl::timer::start(false);
	
	while(true) {
		if (!res.empty()) {
			params.size = res;
			LOG(INFO) << "Camera resolution: " << params.size;
			break;
		}
		sleep(1);
	}

	for (auto *nstream: nstreams) {
		bool res = true;
		while(res) {
			try { res = setRectifyRPC(net, nstream, false); }
			catch (...) {}

			if (res) {
				LOG(ERROR) << "set_rectify() failed for " << *(nstream->get<string>("uri"));
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			else {
				LOG(INFO) << "rectification disabled for " << *(nstream->get<string>("uri"));
			}
		}
	}

	// TODO: parameter for calibration target type
	auto calib = MultiCameraCalibrationNew(	n_cameras, reference_camera,
											params.size, CalibrationTarget(0.250)
	);

	int iter = 0;
	Mat show;

	vector<int> visible;
	vector<vector<Point2d>> points(n_cameras);

	vector<Mat> rgb(n_cameras);
	sleep(3); // rectification disabled, has some delay

	while(calib.getMinVisibility() < n_views) {
		loop:
		cv::waitKey(10);
		
		while (true) {
			if (new_frames) {
				UNIQUE_LOCK(mutex, LOCK);
				rgb.swap(rgb_new);
				new_frames = false;
				break;
			}
			cv::waitKey(10);
		}
		
		for (Mat &im : rgb) {
			if (im.empty()) {
				LOG(ERROR) << "EMPTY";
				goto loop;
			}
		}

		visible.clear();
		int n_found = findCorrespondingPoints(rgb, points, visible);

		if (n_found >= min_visible) {
			calib.addPoints(points, visible);
			
			if (save_input) {
				for (size_t i = 0; i < n_cameras; i++) {
					cv::imwrite(path + std::to_string(i) + "_" + std::to_string(iter) + ".jpg", rgb[i]);
				}
			}
			iter++;
		}

		for (size_t i = 0; i < n_cameras; i++) {
			if (visible[i]) {
				cv::drawMarker(	rgb[i], points[i][0],
								Scalar(42, 255, 42), cv::MARKER_TILTED_CROSS, 25, 2);
				cv::drawMarker(	rgb[i], points[i][1],
								Scalar(42, 42, 255), cv::MARKER_TILTED_CROSS, 25, 2);
			}
			
			// index
			cv::putText(rgb[i],
						"Camera " + std::to_string(i),
						Point2i(10, 30),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(64, 64, 255), 1);
			
			// resolution
			cv::putText(rgb[i],
						"[" + std::to_string(rgb[i].size().width) + "x" + std::to_string(rgb[i].size().height) + "]",
						Point2i(rgb[i].size().width-150, 30),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(64, 64, 255), 1);

			// uri
			cv::putText(rgb[i],
						stream_uris[i/2],
						Point2i(10, rgb[i].rows-10),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(64, 64, 255), 1);

			// remaining frames
			cv::putText(rgb[i],
						std::to_string(std::max(0, (int) (n_views - calib.getViewsCount(i)))),
						Point2i(rgb[i].size().width-150, rgb[i].rows-10),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(64, 64, 255), 1);

		}

		stack(rgb, show);
		cv::namedWindow("Cameras", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Cameras", show);
	}
	cv::destroyWindow("Cameras");
	
	for (size_t i = 0; i < nstreams.size(); i++) {
		while(true) {
			try {
				vector<Mat> D = getDistortionParametersRPC(net, nstreams[i]);
				LOG(INFO) << "K[" << 2*i << "] = \n" << camera_parameters[2*i];
				LOG(INFO) << "D[" << 2*i << "] = " << D[0];
				LOG(INFO) << "K[" << 2*i+1 << "] = \n" << camera_parameters[2*i+1];
				LOG(INFO) << "D[" << 2*i+1 << "] = " << D[1];
				calib.setCameraParameters(2*i, camera_parameters[2*i], D[0]);
				calib.setCameraParameters(2*i+1, camera_parameters[2*i+1], D[1]);
				break;
			}
			catch (...) {}
		}
	}
	
	Mat out;
	vector<Mat> map1, map2;
	vector<cv::Rect> roi;
	vector<size_t> idx;
	calibrateRPC(net, calib, params, nstreams, map1, map2, roi);

	if (save_input) {
		cv::FileStorage fs(path + filename, cv::FileStorage::WRITE);
		calib.saveInput(fs);
		fs.release();
	}


	// visualize
	while(cv::waitKey(10) != 27) {

		while (!new_frames) {
			if (cv::waitKey(50) != -1) { ftl::running = false; }
		}

		{
			UNIQUE_LOCK(mutex, LOCK)
			rgb.swap(rgb_new);
			new_frames = false;
		}

		visualizeCalibration(calib, out, rgb, map1, map2, roi);
		cv::namedWindow("Calibration", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Calibration", out);
	}

	for (size_t i = 0; i < nstreams.size(); i++) {
		while(true) {
			try {
				setRectifyRPC(net, nstreams[i], true);
				break;
			}
			catch (...) {}
		}
	}

	ftl::running = false;
	ftl::timer::stop();
	ftl::pool.stop(true);
}

int main(int argc, char **argv) {
	auto options = ftl::config::read_options(&argv, &argc);
	auto root = ftl::configure(argc, argv, "registration_default");
	
	// run calibration from saved input?
	const bool load_input = root->value<bool>("load_input", false);
	// should calibration input be saved
	const bool save_input = root->value<bool>("save_input", false);
	// should extrinsic calibration be saved (only used with load_input)
	const bool save_extrinsic = root->value<bool>("save_extrinsic", true);
	// should intrinsic calibration be saved
	const bool save_intrinsic = root->value<bool>("save_intrinsic", false);
	const bool optimize_intrinsic = root->value<bool>("optimize_intrinsic", false);
	// directory where calibration data and images are saved, if save_input enabled
	const string calibration_data_dir = root->value<string>("calibration_data_dir", "./");
	// file to save calibration input (2d points and visibility)
	const string calibration_data_file = root->value<string>("calibration_data_file", "data.yml");
	// in how many cameras should the pattern be visible
	const int min_visible = root->value<int>("min_visible", 3);
	// minimum for how many times pattern is seen per camera
	const int n_views = root->value<int>("n_views", 500);
	// reference camera, -1 for automatic
	const int ref_camera = root->value<int>("reference_camera", -1);
	// registration file path
	const string registration_file = root->value<string>("registration_file", FTL_LOCAL_CONFIG_ROOT "/registration.json");
	// location where extrinsic calibration files saved
	const string output_directory = root->value<string>("output_directory", "./");
	
	CalibrationParams params;
	params.save_extrinsic = save_extrinsic;
	params.save_intrinsic = save_intrinsic;
	params.optimize_intrinsic = optimize_intrinsic;
	params.output_path = output_directory;
	params.registration_file = registration_file;
	params.reference_camera = ref_camera;
	
	LOG(INFO)	<< "\n"
				<< "\n"
				<< "\n                save_input: " << (int) save_input
//				<< "\n                load_input: " << (int) load_input
//				<< "\n            save_extrinsic: " << (int) save_extrinsic
//				<< "\n            save_intrinsic: " << (int) save_intrinsic
				<< "\n        optimize_intrinsic: " << (int) optimize_intrinsic
//				<< "\n      calibration_data_dir: " << calibration_data_dir
//				<< "\n     calibration_data_file: " << calibration_data_file
				<< "\n               min_visible: " << min_visible
				<< "\n                   n_views: " << n_views
				<< "\n          reference_camera: " << ref_camera << (ref_camera != -1 ? "" : " (automatic)")
//				<< "\n         registration_file: " << registration_file
//				<< "\n          output_directory: " << output_directory
				<< "\n";

	if (load_input) {
		LOG(FATAL) << "TODO";
	}
	else {
		runCameraCalibration(root, n_views, min_visible, calibration_data_dir, calibration_data_file, save_input, params);
	}

	return 0;
}