/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#include <glog/logging.h>
#include <ftl/config.h>
#include <ftl/configuration.hpp>

// #include <zlib.h>
// #include <lz4.h>

#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/display.hpp>
#include <nlohmann/json.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <ftl/utility/opencv_to_pcl.hpp>
#include <ftl/registration.hpp>

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

#ifdef HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/uniform_sampling.h>
#endif

using ftl::net::Universe;
using ftl::Display;
using ftl::config;
using std::string;
using std::vector;

using json = nlohmann::json;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::mutex;
using std::unique_lock;

using std::vector;

#ifdef HAVE_PCL
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGB;
#endif

using cv::Mat;

class InputStereo {
private:
	string uri_;
	Mat rgb_, disp_;
	Mat q_;
	std::mutex m_;

#ifdef HAVE_PCL
	static PointCloud<PointXYZRGB>::Ptr _getPointCloud(Mat rgb, Mat disp, Mat q) {
		cv::Mat_<cv::Vec3f> XYZ(disp.rows, disp.cols);
		reprojectImageTo3D(disp, XYZ, q, false);
		return ftl::utility::matToPointXYZ(XYZ, rgb);
	}
#endif

public:
	InputStereo(string uri, const Mat Q): uri_(uri), q_(Q) {};
	
	void set(Mat &rgb, Mat &disp) {
		unique_lock<mutex> lk(m_);
		rgb_ = rgb;
		disp_ = disp;
	}
	
	string getURI() { return uri_; }
	Mat getQ() { return q_; }

	/* thread unsafe, use lock() */
#ifdef HAVE_PCL	
	PointCloud<PointXYZRGB>::Ptr getPointCloud() { return _getPointCloud(rgb_, disp_, q_); }
#endif
	Mat getRGB() { return rgb_; }
	Mat getDisparity() { return disp_; }
	/* Mat getDepth() {} */
	/* Mat getLeftRGB() {} */
	/* Mat getRightRGB() {} */
	unique_lock<mutex> lock() { return unique_lock<mutex>(m_); } // use recursive mutex instead (and move locking to methods)?
};

#ifdef HAVE_PCL
#include <pcl/filters/uniform_sampling.h>

/*
class InputMerged {
private:
	// todo: Abstract input systems, can also use other 3D inputs (like InputMerged)
	vector<InputStereo> inputs_;
	vector<Eigen::Matrix4f> T_;
	
public:
	PointCloud<PointXYZRGB>::Ptr getPointCloud() {
		PointCloud<PointXYZRGB>::Ptr result(new PointCloud<PointXYZRGB>);
		for (size_t i = 0; i < T_.size(); i++) {
			inputs_[i].lock();
			PointCloud<PointXYZRGB>::Ptr cloud = inputs_[i].getPointCloud();
			// Documentation: Can be used with cloud_in equal to cloud_out 
			pcl::transformPointCloud(*cloud, *cloud, T_[i]);
			*result += *cloud;
		}
		
		PointCloud<PointXYZRGB>::Ptr result_sampled(new PointCloud<PointXYZRGB>);
		pcl::UniformSampling<PointXYZRGB> uniform_sampling;
		uniform_sampling.setInputCloud(result);
		uniform_sampling.setRadiusSearch(0.1f); // todo parametrize
		uniform_sampling.filter(*result_sampled);
		
		return result_sampled;
	}
};
*/
std::map<string, Eigen::Matrix4f> loadRegistration() {
	std::map<string, Eigen::Matrix4f> registration;
	std::ifstream file(string(FTL_LOCAL_CONFIG_ROOT) + "/registration.json");

	// Use identity transform if no registration
	if (!file.is_open()) {
		Eigen::Matrix4f T;
		registration["default"] = T.setIdentity();
		return registration;
	}

	nlohmann::json load;
	file >> load;
	
	for (auto it = load.begin(); it != load.end(); ++it) {
		Eigen::Matrix4f m;
		auto data = m.data();
		
		for(size_t i = 0; i < 16; i++) {;
			data[i] = it.value()[i];
		}
		
		registration[it.key()] = m;
		
	}
	return registration;
}

void saveRegistration(std::map<string, Eigen::Matrix4f> registration) {
	nlohmann::json save;
	for (auto &item : registration) {
		auto val = nlohmann::json::array();
		for(size_t j = 0; j < 16; j++) { val.push_back((float) item.second.data()[j]); }
		save[item.first] = val;
	}

	std::ofstream file(string(FTL_LOCAL_CONFIG_ROOT) + "/registration.json");
	file << save;
}

template <template<class> class Container>
std::map<string, Eigen::Matrix4f> runRegistration(std::net::Universe &net, Container<InputStereo> &inputs) {
	
	std::map<string, Eigen::Matrix4f> registration;
	
	// NOTE: uses config["registration"]
	
	if (!config["registration"].is_object()) {
		LOG(FATAL) << "Configuration missing \"registration\" entry!";
		return registration;
	}
	
	int iter = (int) config["registration"]["calibration"]["iterations"];
	int delay = (int) config["registration"]["calibration"]["delay"];
	string ref_uri = (string) config["registration"]["reference-source"];
	cv::Size pattern_size(	(int) config["registration"]["calibration"]["patternsize"][0],
							(int) config["registration"]["calibration"]["patternsize"][1]);
	
	// config["registration"] done
	
	size_t ref_i;
	bool found = false;
	for (size_t i = 0; i < inputs.size(); i++) {
		if (inputs[i].getURI() == ref_uri) {
			ref_i = i;
			found = true;
			break;
		}
	}
	
	if (!found) { LOG(ERROR) << "Reference input not found!"; return registration; }
	
	for (auto &input : inputs) { 
		cv::namedWindow("Registration: " + input.getURI(), cv::WINDOW_KEEPRATIO|cv::WINDOW_NORMAL);
	}
	
	// vector for every input: vector of point clouds of patterns
	vector<vector<PointCloud<PointXYZ>::Ptr>> patterns(inputs.size());
	
	while (iter > 0) {
		net.broadcast("grab");
		bool retval = true; // set to false if pattern not found in one of the sources
		vector<PointCloud<PointXYZ>::Ptr> patterns_iter(inputs.size());
		
		for (size_t i = 0; i < inputs.size(); i++) {
			InputStereo &input = inputs[i];
			Mat rgb, disp, Q;
			
			auto lk = input.lock();
			rgb = input.getRGB().clone();
			disp = input.getDisparity().clone();
			Q = input.getQ();
			lk.unlock();
			
			if ((rgb.cols == 0) || (disp.cols == 0)) { retval = false; break; }
			
			retval &= ftl::registration::findChessboardCorners(rgb, disp, Q, pattern_size, patterns_iter[i]);
			
			cv::imshow("Registration: " + input.getURI(), rgb);
		}
		cv::waitKey(delay);
		
		// every camera found the pattern
		if (retval) {
			for (size_t i = 0; i < patterns_iter.size(); i++) {
				patterns[i].push_back(patterns_iter[i]);
			}
			iter--;
		}
		else { LOG(WARNING) << "Pattern not detected on all inputs";}
	}
	
	for (auto &input : inputs) { cv::destroyWindow("Registration: " + input.getURI()); }
	
	for (size_t i = 0; i < inputs.size(); i++) {
		Eigen::Matrix4f T;
		if (i == ref_i) {
			T.setIdentity();
		}
		else {
			T = ftl::registration::findTransformation(patterns[i], patterns[ref_i]);
		}
		registration[inputs[i].getURI()] = T;
	}
	saveRegistration(registration);
	return registration;
}
#endif

bool getCalibration(Universe &net, string src, Mat &Q) {
	Q = Mat(cv::Size(4,4), CV_32F);
	while(true) {
		auto buf = net.findOne<vector<unsigned char>>((string) src +"/calibration");
		if (buf) {
			memcpy(Q.data, (*buf).data(), (*buf).size());
			
			if (Q.step*Q.rows != (*buf).size()) {
				LOG(ERROR) << "Corrupted calibration";
				return false;
			}
			
			return true;
		}
		else {
			LOG(INFO) << "Could not get calibration, retrying";
			sleep_for(milliseconds(500));
		}
	}
}

static void run() {
	Universe net(config["net"]);
	
	// Make sure connections are complete
	sleep_for(milliseconds(500));
	
	if (!config["sources"].is_array()) {
		LOG(ERROR) << "No sources configured!";
		return;
	}
	
	std::deque<InputStereo> inputs;
	std::vector<Display> displays;

	// todo networking and initilization for input should possibly be implemented in their own class
	//      with error handling etc.
	//
	
	for (auto &src : config["sources"]) {
		Mat Q;
		if (!getCalibration(net, src, Q)) {	continue; } // skips input if calibration bad
		
		InputStereo &in = inputs.emplace_back(src, Q);
		displays.emplace_back(config["display"], src);
		
		LOG(INFO) << src << " loaded";
		
		net.subscribe(src, [&in](const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
			Mat rgb, disp;
			cv::imdecode(jpg, cv::IMREAD_COLOR, &rgb);
			Mat(rgb.size(), CV_16UC1);
			cv::imdecode(d, cv::IMREAD_UNCHANGED, &disp);
			disp.convertTo(disp, CV_32FC1, 1.0f/16.0f);
			in.set(rgb, disp);
		});
	}
	
	// Displays and Inputs configured
	
	// load point cloud transformations
	
#ifdef HAVE_PCL
	std::map<string, Eigen::Matrix4f> registration;
	if (config["registration"]["calibration"]["run"]) {
		registration = runRegistration(net, inputs);
	}
	else {
		registration = loadRegistration();
	}
	vector<Eigen::Matrix4f> T;
	for (auto &input : inputs) {
		Eigen::Matrix4f RT = (registration.count(input.getURI()) > 0) ? registration[input.getURI()] : registration["default"];
		T.push_back(RT);
	}
	
	//
	vector<PointCloud<PointXYZRGB>::Ptr> clouds(inputs.size());
	Display display_merged(config["display"], "Merged"); // todo

	int active = displays.size();
	while (active > 0) {
		active = 0;

		net.broadcast("grab");  // To sync cameras
		
		PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
		
		for (size_t i = 0; i < inputs.size(); i++) {
			Display &display = displays[i];
			InputStereo &input = inputs[i];
			
			if (!display.active()) continue;
			active += 1;
			
			auto lk = input.lock();
			//Mat rgb = input.getRGB();
			//Mat disparity = input.getDisparity();
			clouds[i] = input.getPointCloud();
			lk.unlock();
			
			display.render(clouds[i]);
			//display.render(rgb, disparity);
			display.wait(50);
		}
		
		for (size_t i = 0; i < clouds.size(); i++) {
			pcl::transformPointCloud(*clouds[i], *clouds[i], T[i]);
			*cloud += *clouds[i];
		}
		
		pcl::UniformSampling<PointXYZRGB> uniform_sampling;
		uniform_sampling.setInputCloud(cloud);
		uniform_sampling.setRadiusSearch(0.1f);
		uniform_sampling.filter(*cloud);
		
		display_merged.render(cloud);
		display_merged.wait(50);
	}
#endif
	// TODO non-PCL (?)
}

int main(int argc, char **argv) {
	ftl::configure(argc, argv, "representation"); // TODO(nick) change to "reconstruction"
	run();
}
