#include <loguru.hpp>

#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/group.hpp>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

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

Mat getCameraMatrix(const ftl::rgbd::Camera &parameters) {
	Mat m = (cv::Mat_<double>(3,3) << parameters.fx, 0.0, -parameters.cx, 0.0, parameters.fy, -parameters.cy, 0.0, 0.0, 1.0);
	return m;
}

void to_json(nlohmann::json &json, map<string, Eigen::Matrix4d> &transformations) {
	for (auto &item : transformations) {
		auto val = nlohmann::json::array();
		for(size_t i = 0; i < 16; i++) { val.push_back((float) item.second.data()[i]); }
		json[item.first] = val;
	}
}

// FileStorage allows only alphanumeric keys (code below does not work with URIs)

bool saveRegistration(const string &ofile, const map<string, Mat> &data) {
	cv::FileStorage fs(ofile, cv::FileStorage::WRITE);
	if (!fs.isOpened()) return false;
	for (auto &item : data) { fs << item.first << item.second; }
	fs.release();
	return true;
}

bool saveRegistration(const string &ofile, const map<string, Eigen::Matrix4d> &data) {
	map<string, Mat> _data;
	for (auto &item : data) {
		Mat M;
		cv::eigen2cv(item.second, M);
		_data[item.first] = M; 
	}
	return saveRegistration(ofile, _data);
}

bool loadRegistration(const string &ifile, map<string, Mat> &data) {
	cv::FileStorage fs(ifile, cv::FileStorage::READ);
	if (!fs.isOpened()) return false;
	for(cv::FileNodeIterator fit = fs.getFirstTopLevelNode().begin();
		fit != fs.getFirstTopLevelNode().end();
		++fit)
	{
		data[(*fit).name()] = (*fit).mat();
	}
	fs.release();
	return true; // TODO errors?
}

bool loadRegistration(const string &ifile, map<string, Eigen::Matrix4d> &data) {
	map<string, Mat> _data;
	if (!loadRegistration(ifile, _data)) return false;
	for (auto &item : _data) {
		Eigen::Matrix4d M;
		cv::cv2eigen(item.second, M);
		data[item.first] = M;
	}
	return true;
}

//

bool saveIntrinsics(const string &ofile, const vector<Mat> &M) {
	vector<Mat> D;
	{
		cv::FileStorage fs(ofile, cv::FileStorage::READ);
		fs["D"] >> D;
		fs.release();
	}
	{
		cv::FileStorage fs(ofile, cv::FileStorage::WRITE);
		if (fs.isOpened()) {
			fs << "K" << M << "D" << D;
			fs.release();
			return true;
		}
		else {
			LOG(ERROR) << "Error: can not save the intrinsic parameters to '" << ofile << "'";
		}
		return false;
	}
}

bool saveExtrinsics(const string &ofile, Mat &R, Mat &T, Mat &R1, Mat &R2, Mat &P1, Mat &P2, Mat &Q) {
	cv::FileStorage fs;
	fs.open(ofile, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1"
			<< P1 << "P2" << P2 << "Q" << Q;
		fs.release();
		return true;
	} else {
		LOG(ERROR) << "Error: can not save the extrinsic parameters";
	}
	return false;
}

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

string time_now_string() {
	char timestamp[18];
	std::time_t t=std::time(NULL);
	std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
	return string(timestamp);
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
	}

	stack(rgb, out);
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
};

void calibrate(	MultiCameraCalibrationNew &calib, vector<string> &uri_cameras,
				const CalibrationParams &params, vector<Mat> &map1, vector<Mat> &map2, vector<cv::Rect> &roi)
{
	int reference_camera = -1;
	if (params.reference_camera < 0) {
		reference_camera = calib.getOptimalReferenceCamera();
		reference_camera -= (reference_camera & 1);
	}
	LOG(INFO) << "optimal camera: " << reference_camera;

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
		cv::stereoRectify(K1, D1, K2, D2, Size(1280, 720), R_c1c2, T_c1c2, R1, R2, P1, P2, Q, 0, params.alpha);
		
		Mat _t = Mat(Size(1, 3), CV_64FC1, Scalar(0.0));
		Rt_out[c] = getMat4x4(R[c], t[c]) * getMat4x4(R1, _t).inv();
		Rt_out[c + 1] = getMat4x4(R[c + 1], t[c + 1]) * getMat4x4(R2, _t).inv();

		{
			string node_name;
			size_t pos1 = uri_cameras[c/2].find("node");
			size_t pos2 = uri_cameras[c/2].find("#", pos1);
			node_name = uri_cameras[c/2].substr(pos1, pos2 - pos1);
			//LOG(INFO) << c << ":" << calib.getCameraMatNormalized(c, 1280, 720);
			//LOG(INFO) << c + 1 << ":" << calib.getCameraMatNormalized(c + 1, 1280, 720);
			if (params.save_extrinsic) {
				saveExtrinsics(params.output_path + node_name + "-extrinsic.yml", R_c1c2, T_c1c2, R1, R2, P1, P2, Q);
				LOG(INFO) << "Saved: " << params.output_path + node_name + "-extrinsic.yml";
			}
			if (params.save_intrinsic) {
				saveIntrinsics(params.output_path + node_name + "-intrinsic.yml",
					{calib.getCameraMatNormalized(c, 1280, 720), calib.getCameraMatNormalized(c + 1, 1280, 720)}
				);
				LOG(INFO) << "Saved: " << params.output_path + node_name + "-intrinsic.yml";
			}
		}

		// for visualization
		Size new_size;
		cv::stereoRectify(K1, D1, K2, D2, Size(1280, 720), R_c1c2, T_c1c2, R1, R2, P1, P2, Q, 0, 1.0, new_size, &roi[c], &roi[c + 1]);
		cv::initUndistortRectifyMap(K1, D1, R1, P1, Size(1280, 720), CV_16SC2, map1[c], map2[c]);
		cv::initUndistortRectifyMap(K2, D2, R2, P2, Size(1280, 720), CV_16SC2, map1[c + 1], map2[c + 1]);
	}

	{
		map<string, Eigen::Matrix4d> out;
		for (size_t i = 0; i < n_cameras; i += 2) {
			Eigen::Matrix4d M_eigen;
			Mat M_cv = Rt_out[i];
			cv::cv2eigen(M_cv, M_eigen);
			out[uri_cameras[i/2]] = M_eigen;
		}
		
		nlohmann::json out_json;
		to_json(out_json, out);
		if (params.save_extrinsic) {
			std::ofstream file_out(params.registration_file);
			file_out << out_json;
		}
		else {
			LOG(INFO) << "Registration not saved to file";
			LOG(INFO) << out_json;
		}
	}
}

void calibrateFromPath(	const string &path,
						const string &filename,
						CalibrationParams &params,
						bool visualize=false)
{
	size_t reference_camera = 0;
	auto calib = MultiCameraCalibrationNew(0, reference_camera, Size(0, 0), CalibrationTarget(0.250));
	
	vector<string> uri_cameras;
	cv::FileStorage fs(path + filename, cv::FileStorage::READ);
	fs["uri"] >> uri_cameras;

	//params.idx_cameras = {2, 3};//{0, 1, 4, 5, 6, 7, 8, 9, 10, 11};
	params.idx_cameras.resize(uri_cameras.size() * 2);
	std::iota(params.idx_cameras.begin(), params.idx_cameras.end(), 0);

	calib.loadInput(path + filename, params.idx_cameras);
	
	vector<Mat> map1, map2;
	vector<cv::Rect> roi;
	calibrate(calib, uri_cameras, params, map1, map2, roi);

	if (!visualize) return;

	vector<Scalar> colors = {
		Scalar(64, 64, 255),
		Scalar(64, 64, 255),
		Scalar(64, 255, 64),
		Scalar(64, 255, 64),
	};
	vector<int> markers = {cv::MARKER_SQUARE, cv::MARKER_DIAMOND};

	Mat out;
	size_t n_cameras = calib.getCamerasCount();
	vector<Mat> rgb(n_cameras);
	size_t i = 0;
	while(ftl::running) {
		for (size_t c = 0; c < n_cameras; c++) {
			rgb[c] = cv::imread(path + std::to_string(params.idx_cameras[c]) + "_" + std::to_string(i) + ".jpg");
			for (size_t j = 0; j < rgb.size(); j++) {
				vector<Point2d> points;
				// TODO: indexing incorrect if all cameras used (see also loadInput)
				calib.projectPointsOptimized(c, i, points); // project BA point to image

				for (Point2d &p : points) {
					cv::drawMarker(rgb[c], cv::Point2i(p), colors[j % colors.size()], markers[j % markers.size()], 10 + 3 * j, 1);
				}
			}
		}
		
		visualizeCalibration(calib, out, rgb, map1, map2, roi);	
		cv::namedWindow("Calibration", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Calibration", out);

		i = (i + 1) % calib.getViewsCount();
		
		if (cv::waitKey(50) != -1) { ftl::running = false; }
	}
}

void runCameraCalibration(	ftl::Configurable* root,
							int n_views, int min_visible,
							string path, string filename,
							bool save_input,
							CalibrationParams &params)
{
	Universe *net = ftl::create<Universe>(root, "net");
	net->start();
	net->waitConnections();

	vector<Source*> sources = ftl::createArray<Source>(root, "sources", net);
	
	const size_t n_sources = sources.size();
	const size_t n_cameras = n_sources * 2;
	size_t reference_camera = 0;
	Size resolution;
	{
		auto params = sources[0]->parameters();
		resolution = Size(params.width, params.height);
		LOG(INFO) << "Camera resolution: " << resolution;
	}

	params.idx_cameras.resize(n_cameras);
	std::iota(params.idx_cameras.begin(), params.idx_cameras.end(), 0);

	// TODO: parameter for calibration target type
	auto calib = MultiCameraCalibrationNew(	n_cameras, reference_camera,
											resolution, CalibrationTarget(0.250)
	);

	for (size_t i = 0; i < n_sources; i++) {
		auto params_r = sources[i]->parameters(ftl::rgbd::kChanRight);
		auto params_l = sources[i]->parameters(ftl::rgbd::kChanLeft);

		CHECK(resolution == Size(params_r.width, params_r.height));
		CHECK(resolution == Size(params_l.width, params_l.height));
		
		Mat K;
		K = getCameraMatrix(params_r);
		LOG(INFO) << "K[" << 2 * i + 1 << "] = \n" << K;
		calib.setCameraParameters(2 * i + 1, K);

		K = getCameraMatrix(params_l);
		LOG(INFO) << "K[" << 2 * i << "] = \n" << K;
		calib.setCameraParameters(2 * i, K);
	}

	ftl::rgbd::Group group;
	for (Source* src : sources) {
		src->setChannel(ftl::rgbd::kChanRight);
		group.addSource(src);
	}

	std::mutex mutex;
	std::atomic<bool> new_frames = false;
	vector<Mat> rgb(n_cameras), rgb_new(n_cameras);
	
	ftl::timer::start(false);

	group.sync([&mutex, &new_frames, &rgb_new](ftl::rgbd::FrameSet &frames) {
		mutex.lock();
		bool good = true;
		for (size_t i = 0; i < frames.channel1.size(); i ++) {
			if (frames.channel1[i].empty()) good = false;
			if (frames.channel1[i].empty()) good = false;
			if (frames.channel1[i].channels() != 3) good = false; // ASSERT
			if (frames.channel2[i].channels() != 3) good = false;
			if (!good) break;
			cv::swap(frames.channel1[i], rgb_new[2 * i]);
			cv::swap(frames.channel2[i], rgb_new[2 * i + 1]);
		}

		new_frames = good;
		mutex.unlock();
		return true;
	});
	
	int iter = 0;
	Mat show;

	vector<int> visible;
	vector<vector<Point2d>> points(n_cameras);

	while(calib.getMinVisibility() < n_views) {
		cv::waitKey(10);
		while (!new_frames) {
			for (auto src : sources) { src->grab(30); }
			cv::waitKey(10);
		}

		mutex.lock();
		rgb.swap(rgb_new);
		new_frames = false;
		mutex.unlock();

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
			cv::putText(rgb[i],
						"Camera " + std::to_string(i),
						Point2i(10, 30),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(64, 64, 255), 1);
			
			cv::putText(rgb[i],
						std::to_string(std::max(0, (int) (n_views - calib.getViewsCount(i)))),
						Point2i(10, rgb[i].rows-10),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(64, 64, 255), 1);

		}

		stack(rgb, show);
		cv::namedWindow("Cameras", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Cameras", show);
	}
	cv::destroyWindow("Cameras");

	vector<string> uri;
	for (size_t i = 0; i < n_sources; i++) {
		uri.push_back(sources[i]->getURI());
	}

	if (save_input) {
		cv::FileStorage fs(path + filename, cv::FileStorage::WRITE);
		fs << "uri" << uri;
		calib.saveInput(fs);
		fs.release();
	}

	Mat out;
	vector<Mat> map1, map2;
	vector<cv::Rect> roi;
	vector<size_t> idx;
	calibrate(calib, uri, params, map1, map2, roi);

	// visualize
	while(ftl::running) {
		while (!new_frames) {
			for (auto src : sources) { src->grab(30); }
			if (cv::waitKey(50) != -1) { ftl::running = false; }
		}

		mutex.lock();
		rgb.swap(rgb_new);
		new_frames = false;
		mutex.unlock();

		visualizeCalibration(calib, out, rgb, map1, map2, roi);
		cv::namedWindow("Calibration", cv::WINDOW_KEEPRATIO | cv::WINDOW_NORMAL);
		cv::imshow("Calibration", out);
	}
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

	LOG(INFO)	<< "\n"
				<< "\nIMPORTANT: Remeber to set \"use_intrinsics\" to false for nodes!"
				<< "\n"
				<< "\n                save_input: " << (int) save_input
				<< "\n                load_input: " << (int) load_input
				<< "\n            save_extrinsic: " << (int) save_extrinsic
				<< "\n            save_intrinsic: " << (int) save_intrinsic
				<< "\n        optimize_intrinsic: " << (int) optimize_intrinsic
				<< "\n      calibration_data_dir: " << calibration_data_dir
				<< "\n     calibration_data_file: " << calibration_data_file
				<< "\n               min_visible: " << min_visible
				<< "\n                   n_views: " << n_views
				<< "\n         registration_file: " << registration_file
				<< "\n          output_directory: " << output_directory
				<< "\n";

	if (load_input) {
		vector<size_t> idx = {};
		calibrateFromPath(calibration_data_dir, calibration_data_file, params, true);
	}
	else {
		runCameraCalibration(root, n_views, min_visible, calibration_data_dir, calibration_data_file, save_input, params);
	}

	return 0;
}