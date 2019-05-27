/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#include <glog/logging.h>
#include <ftl/config.h>
#include <ftl/configuration.hpp>
#include <ftl/depth_camera.hpp>
#include <ftl/scene_rep_hash_sdf.hpp>
#include <ftl/ray_cast_sdf.hpp>
#include <ftl/rgbd.hpp>

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
using ftl::rgbd::RGBDSource;

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

namespace ftl {
namespace rgbd {
PointCloud<PointXYZRGB>::Ptr createPointCloud(RGBDSource *src);
PointCloud<PointXYZRGB>::Ptr createPointCloud(RGBDSource *src, const cv::Point3_<uchar> &col);
}
}

PointCloud<PointXYZRGB>::Ptr ftl::rgbd::createPointCloud(RGBDSource *src) {
	const double CX = src->getParameters().cx;
	const double CY = src->getParameters().cy;
	const double FX = src->getParameters().fx;
	const double FY = src->getParameters().fy;

	cv::Mat rgb;
	cv::Mat depth;
	src->getRGBD(rgb, depth);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_ptr->width = rgb.cols * rgb.rows;
	point_cloud_ptr->height = 1;

	for(int i=0;i<rgb.rows;i++) {
		const float *sptr = depth.ptr<float>(i);
		for(int j=0;j<rgb.cols;j++) {
			float d = sptr[j]; // * 1000.0f;

			pcl::PointXYZRGB point;
			point.x = (((double)j + CX) / FX) * d;
			point.y = (((double)i + CY) / FY) * d;
			point.z = d;

			if (point.x == INFINITY || point.y == INFINITY || point.z > 20.0f || point.z < 0.04f) {
				point.x = 0.0f; point.y = 0.0f; point.z = 0.0f;
			}

			cv::Point3_<uchar> prgb = rgb.at<cv::Point3_<uchar>>(i, j);
			uint32_t rgb = (static_cast<uint32_t>(prgb.z) << 16 | static_cast<uint32_t>(prgb.y) << 8 | static_cast<uint32_t>(prgb.x));
			point.rgb = *reinterpret_cast<float*>(&rgb);

			point_cloud_ptr -> points.push_back(point);
		}
	}

	return point_cloud_ptr;
}

PointCloud<PointXYZRGB>::Ptr ftl::rgbd::createPointCloud(RGBDSource *src, const cv::Point3_<uchar> &prgb) {
	const double CX = src->getParameters().cx;
	const double CY = src->getParameters().cy;
	const double FX = src->getParameters().fx;
	const double FY = src->getParameters().fy;

	cv::Mat rgb;
	cv::Mat depth;
	src->getRGBD(rgb, depth);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_ptr->width = rgb.cols * rgb.rows;
	point_cloud_ptr->height = 1;

	for(int i=0;i<rgb.rows;i++) {
		const float *sptr = depth.ptr<float>(i);
		for(int j=0;j<rgb.cols;j++) {
			float d = sptr[j]; // * 1000.0f;

			pcl::PointXYZRGB point;
			point.x = (((double)j + CX) / FX) * d;
			point.y = (((double)i + CY) / FY) * d;
			point.z = d;

			if (point.x == INFINITY || point.y == INFINITY || point.z > 20.0f || point.z < 0.04f) {
				point.x = 0.0f; point.y = 0.0f; point.z = 0.0f;
			}

			uint32_t rgb = (static_cast<uint32_t>(prgb.z) << 16 | static_cast<uint32_t>(prgb.y) << 8 | static_cast<uint32_t>(prgb.x));
			point.rgb = *reinterpret_cast<float*>(&rgb);

			point_cloud_ptr -> points.push_back(point);
		}
	}

	return point_cloud_ptr;
}


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

struct Cameras {
	RGBDSource *source;
	DepthCameraData gpu;
	DepthCameraParams params;
};

template <template<class> class Container>
std::map<string, Eigen::Matrix4f> runRegistration(ftl::net::Universe &net, Container<Cameras> &inputs) {
	
	std::map<string, Eigen::Matrix4f> registration;
	
	// NOTE: uses config["registration"]
	
	if (!config["registration"].is_object()) {
		LOG(FATAL) << "Configuration missing \"registration\" entry!";
		return registration;
	}
	
	int iter = (int) config["registration"]["calibration"]["iterations"];
	int delay = (int) config["registration"]["calibration"]["delay"];
	float max_error = (int) config["registration"]["calibration"]["max_error"];
	string ref_uri = (string) config["registration"]["reference-source"];
	cv::Size pattern_size(	(int) config["registration"]["calibration"]["patternsize"][0],
							(int) config["registration"]["calibration"]["patternsize"][1]);
	
	// config["registration"] done
	
	size_t ref_i;
	bool found = false;
	for (size_t i = 0; i < inputs.size(); i++) {
		if (inputs[i].source->getConfig()["uri"] == ref_uri) {
			ref_i = i;
			found = true;
			break;
		}
	}

	if (!found) { LOG(ERROR) << "Reference input not found!"; return registration; }
	
	for (auto &input : inputs) { 
		cv::namedWindow("Registration: " + (string)input.source->getConfig()["uri"], cv::WINDOW_KEEPRATIO|cv::WINDOW_NORMAL);
	}
	
	// vector for every input: vector of point clouds of patterns
	vector<vector<PointCloud<PointXYZ>::Ptr>> patterns(inputs.size());
	
	while (iter > 0) {
		net.broadcast("grab");
		bool retval = true; // set to false if pattern not found in one of the sources
		vector<PointCloud<PointXYZ>::Ptr> patterns_iter(inputs.size());
		
		for (size_t i = 0; i < inputs.size(); i++) {
			RGBDSource *input = inputs[i].source;
			Mat rgb, depth;
			
			input->getRGBD(rgb, depth);
			
			if ((rgb.cols == 0) || (depth.cols == 0)) { retval = false; break; }
			
			retval &= ftl::registration::findChessboardCorners(rgb, depth, input->getParameters(), pattern_size, patterns_iter[i], max_error);
			
			cv::imshow("Registration: " + (string)input->getConfig()["uri"], rgb);
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
	
	for (auto &input : inputs) { cv::destroyWindow("Registration: " + (string)input.source->getConfig()["uri"]); }
	
	for (size_t i = 0; i < inputs.size(); i++) {
		Eigen::Matrix4f T;
		if (i == ref_i) {
			T.setIdentity();
		}
		else {
			T = ftl::registration::findTransformation(patterns[i], patterns[ref_i]);
		}
		registration[(string)inputs[i].source->getConfig()["uri"]] = T;
	}
	saveRegistration(registration);
	return registration;
}
#endif

template<class T>
Eigen::Matrix<T,4,4> lookAt
(
	Eigen::Matrix<T,3,1> const & eye,
	Eigen::Matrix<T,3,1> const & center,
	Eigen::Matrix<T,3,1> const & up
)
{
	typedef Eigen::Matrix<T,4,4> Matrix4;
	typedef Eigen::Matrix<T,3,1> Vector3;

	Vector3 f = (center - eye).normalized();
	Vector3 u = up.normalized();
	Vector3 s = f.cross(u).normalized();
	u = s.cross(f);

	Matrix4 res;
	res <<	s.x(),s.y(),s.z(),-s.dot(eye),
			u.x(),u.y(),u.z(),-u.dot(eye),
			-f.x(),-f.y(),-f.z(),f.dot(eye),
			0,0,0,1;

	return res;
}

using MouseAction = std::function<void(int, int, int, int)>;

static void setMouseAction(const std::string& winName, const MouseAction &action)
{
  cv::setMouseCallback(winName,
                       [] (int event, int x, int y, int flags, void* userdata) {
    (*(MouseAction*)userdata)(event, x, y, flags);
  }, (void*)&action);
}

static void run() {
	Universe net(config["net"]);
	
	// Make sure connections are complete
	sleep_for(milliseconds(500));
	
	if (!config["sources"].is_array()) {
		LOG(ERROR) << "No sources configured!";
		return;
	}
	
	std::deque<Cameras> inputs;
	//std::vector<Display> displays;

	// TODO Allow for non-net source types
	for (auto &src : config["sources"]) {		
		RGBDSource *in = ftl::rgbd::RGBDSource::create(src, &net); //new ftl::rgbd::NetSource(src, &net);
		if (!in) {
			LOG(ERROR) << "Unrecognised source: " << src;
		} else {
			auto &cam = inputs.emplace_back();
			cam.source = in;
			cam.params.fx = in->getParameters().fx;
			cam.params.fy = in->getParameters().fy;
			cam.params.mx = -in->getParameters().cx;
			cam.params.my = -in->getParameters().cy;
			cam.params.m_imageWidth = in->getParameters().width;
			cam.params.m_imageHeight = in->getParameters().height;
			cam.params.m_sensorDepthWorldMax = in->getParameters().maxDepth;
			cam.params.m_sensorDepthWorldMin = in->getParameters().minDepth;
			cam.gpu.alloc(cam.params);
			
			//displays.emplace_back(config["display"], src["uri"]);
			LOG(INFO) << (string)src["uri"] << " loaded " << cam.params.m_imageWidth;
		}
	}
	
	// Displays and Inputs configured
	
	// load point cloud transformations
	
	std::map<string, Eigen::Matrix4f> registration;
	if (config["registration"]["calibration"]["run"]) {
		registration = runRegistration(net, inputs);
	}
	else {
		LOG(INFO) << "LOAD REG";
		registration = loadRegistration();
	}

	LOG(INFO) << "Assigning poses";
	vector<Eigen::Matrix4f> T;
	for (auto &input : inputs) {
		LOG(INFO) << (unsigned long long)input.source;
		Eigen::Matrix4f RT = (registration.count(input.source->getConfig()["uri"].get<string>()) > 0) ? registration[(string)input.source->getConfig()["uri"]] : registration["default"];
		T.push_back(RT);
		input.source->setPose(RT);
	}
	
	//vector<PointCloud<PointXYZRGB>::Ptr> clouds(inputs.size());
	Display display_merged(config["display"], "Merged"); // todo
	CUDARayCastSDF rays(config["voxelhash"]);
	ftl::voxhash::SceneRep scene(config["voxelhash"]);

	cv::Mat colour_array(cv::Size(rays.getRayCastParams().m_width,rays.getRayCastParams().m_height), CV_8UC3);

	// Force window creation
	display_merged.render(colour_array);
	display_merged.wait(1);

	unsigned char frameCount = 0;
	bool paused = false;
	float cam_x = 0.0f;
	float cam_z = 0.0f;

	Eigen::Vector3f eye(0.0f, 0.0f, 0.0f);
	Eigen::Vector3f centre(0.0f, 0.0f, -4.0f);
	Eigen::Vector3f up(0,1.0f,0);
	Eigen::Vector3f lookPoint(0.0f,1.0f,-4.0f);
	Eigen::Matrix4f viewPose;
	float lerpSpeed = 0.4f;

	// Keyboard camera controls
	display_merged.onKey([&paused,&eye,&centre](int key) {
		LOG(INFO) << "Key = " << key;
		if (key == 32) paused = !paused;
		else if (key == 81) eye[0] += 0.02f;
		else if (key == 83) eye[0] -= 0.02f;
		else if (key == 84) eye[2] += 0.02f;
		else if (key == 82) eye[2] -= 0.02f;
	});

	// TODO(Nick) Calculate "camera" properties of viewport.
	MouseAction mouseact = [&inputs,&lookPoint,&viewPose]( int event, int ux, int uy, int) {
		LOG(INFO) << "Mouse " << ux << "," << uy;
		if (event == 1) {   // click
			const float x = ((float)ux-inputs[0].params.mx) / inputs[0].params.fx;
			const float y = ((float)uy-inputs[0].params.my) / inputs[0].params.fy;
			const float depth = -4.0f;
			Eigen::Vector4f camPos(x*depth,y*depth,depth,1.0);
			Eigen::Vector4f worldPos =  viewPose * camPos;
			lookPoint = Eigen::Vector3f(worldPos[0],worldPos[1],worldPos[2]);
		}
	};
	::setMouseAction("Image", mouseact);

	int active = inputs.size();
	while (active > 0 && display_merged.active()) {
		active = 0;

		if (!paused) {
			net.broadcast("grab");  // To sync cameras
			scene.nextFrame();
		
			for (size_t i = 0; i < inputs.size(); i++) {
				//if (i == 1) continue;
				//Display &display = displays[i];
				RGBDSource *input = inputs[i].source;
				Mat rgb, depth;
				//LOG(INFO) << "GetRGB";
				input->getRGBD(rgb,depth);
				
				//if (!display.active()) continue;
				active += 1;

				//clouds[i] = ftl::rgbd::createPointCloud(input);
				
				//display.render(rgb, depth,input->getParameters());
				//display.render(clouds[i]);
				//display.wait(5);

				//LOG(INFO) << "Data size: " << depth.cols << "," << depth.rows;
				if (depth.cols == 0) continue;

				Mat rgba;
				cv::cvtColor(rgb,rgba, cv::COLOR_BGR2BGRA);

				inputs[i].params.flags = frameCount;

				// Send to GPU and merge view into scene
				inputs[i].gpu.updateParams(inputs[i].params);
				inputs[i].gpu.updateData(depth, rgba);
				scene.integrate(inputs[i].source->getPose(), inputs[i].gpu, inputs[i].params, nullptr);
			}
		} else {
			active = 1;
		}

		frameCount++;

		// Set virtual camera transformation matrix
		//Eigen::Affine3f transform(Eigen::Translation3f(cam_x,0.0f,cam_z));
		centre += (lookPoint - centre) * (lerpSpeed * 0.1f);
		viewPose = lookAt<float>(eye,centre,up); // transform.matrix();

		// Call GPU renderer and download result into OpenCV mat
		rays.render(scene.getHashData(), scene.getHashParams(), inputs[0].gpu, viewPose);
		rays.getRayCastData().download(nullptr, (uchar3*)colour_array.data, rays.getRayCastParams());
		display_merged.render(colour_array);
		display_merged.wait(1);
	}
}

int main(int argc, char **argv) {
	ftl::configure(argc, argv, "reconstruction");
	run();
}
