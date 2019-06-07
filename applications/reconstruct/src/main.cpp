/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#define LOGURU_WITH_STREAMS 1
#include <loguru.hpp>
#include <ftl/config.h>
#include <ftl/configuration.hpp>
#include <ftl/depth_camera.hpp>
#include <ftl/scene_rep_hash_sdf.hpp>
#include <ftl/rgbd.hpp>
#include <ftl/virtual_source.hpp>
#include <ftl/rgbd_streamer.hpp>

// #include <zlib.h>
// #include <lz4.h>

#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/rgbd_display.hpp>
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
using ftl::rgbd::Display;
using std::string;
using std::vector;
using ftl::rgbd::RGBDSource;
using ftl::config::json_t;

using json = nlohmann::json;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::mutex;
using std::unique_lock;

using std::vector;

using cv::Mat;

using ftl::registration::loadTransformations;
using ftl::registration::saveTransformations;

struct Cameras {
	RGBDSource *source;
	DepthCameraData gpu;
	DepthCameraParams params;
};

static void run(ftl::Configurable *root) {
	Universe *net = ftl::create<Universe>(root, "net");
	
	// Make sure connections are complete
	// sleep_for(milliseconds(500));
	// TODO: possible to do more reliably?
	net->waitConnections();
	
	std::vector<Cameras> inputs;
	auto sources = root->get<vector<json_t>>("sources");

	if (!sources) {
		LOG(ERROR) << "No sources configured!";
		return;
	}

	// TODO Allow for non-net source types
	for (auto &src : *sources) {
		RGBDSource *in = ftl::rgbd::RGBDSource::create(src, net);
		if (!in) {
			LOG(ERROR) << "Unrecognized source: " << src;
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
			
			LOG(INFO) << (string) src["uri"] << " loaded " << cam.params.fx;
		}
	}

	// TODO	move this block in its own method and add automated tests
	//		for error cases
	
	std::optional<json_t> merge = root->get<json_t>("merge");
	if (!merge) {
		LOG(WARNING) << "Input merging not configured, using only first input in configuration";
		inputs = { inputs[0] };
	}

	if (inputs.size() > 1) {
		std::map<std::string, Eigen::Matrix4f> transformations;

		if ((*merge)["register"]) {
			LOG(INFO) << "Registration requested";

			ftl::registration::Registration *reg = ftl::registration::ChessboardRegistration::create(*merge);
			for (auto &input : inputs) { 
				while(!input.source->isReady()) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
				reg->addSource(input.source);
			}
			
			reg->run();
			if (reg->findTransformations(transformations)) {
				if (!saveTransformations(string(FTL_LOCAL_CONFIG_ROOT) + "/registration.json", transformations)) {
					LOG(ERROR) << "Error saving new registration";
				};
			}
			else {
				LOG(ERROR) << "Registration failed";
			}

			free(reg);
		}
		else {
			if (loadTransformations(string(FTL_LOCAL_CONFIG_ROOT) + "/registration.json", transformations)) {
				LOG(INFO) << "Loaded camera trasformations from file";
			}
			else {
				LOG(ERROR) << "Error loading camera transformations from file";
			}
		}

		for (auto &input : inputs) {
			string uri = input.source->getURI();
			auto T = transformations.find(uri);
			if (T == transformations.end()) {
				LOG(ERROR) << "Camera pose for " + uri + " not found in transformations";
				LOG(WARNING) << "Using only first configured source";
				// TODO: use target source if configured and found
				inputs = { inputs[0] };
				inputs[0].source->setPose(Eigen::Matrix4f::Identity());
				break;
			}
			input.source->setPose(T->second);
		}
	}

	// Displays and Inputs configured
	
	LOG(INFO) << "Using sources:";
	for (auto &input : inputs) { LOG(INFO) << "    " + (string) input.source->getURI(); }
	
	ftl::rgbd::Display *display = ftl::create<ftl::rgbd::Display>(root, "display");
	ftl::rgbd::VirtualSource *virt = ftl::create<ftl::rgbd::VirtualSource>(root, "virtual", net);
	ftl::voxhash::SceneRep *scene = ftl::create<ftl::voxhash::SceneRep>(root, "voxelhash");
	virt->setScene(scene);
	display->setSource(virt);

	ftl::rgbd::Streamer *stream = ftl::create<ftl::rgbd::Streamer>(root, "stream", net);
	stream->add(virt);
	// Also proxy all inputs
	for (auto &in : inputs) {
		stream->add(in.source);
	}

	unsigned char frameCount = 0;
	bool paused = false;

	// Keyboard camera controls
	display->onKey([&paused](int key) {
		if (key == 32) paused = !paused;
	});

	int active = inputs.size();
	while (active > 0 && display->active()) {
		active = 0;

		if (!paused) {
			//net.broadcast("grab");  // To sync cameras
			scene->nextFrame();
		
			for (size_t i = 0; i < inputs.size(); i++) {
				// Get the RGB-Depth frame from input
				RGBDSource *input = inputs[i].source;
				Mat rgb, depth;
				input->grab();
				input->getRGBD(rgb,depth);
				
				active += 1;

				if (depth.cols == 0) continue;

				// Must be in RGBA for GPU
				Mat rgba;
				cv::cvtColor(rgb,rgba, cv::COLOR_BGR2BGRA);

				inputs[i].params.flags = frameCount;

				// Send to GPU and merge view into scene
				inputs[i].gpu.updateParams(inputs[i].params);
				inputs[i].gpu.updateData(depth, rgba);
				scene->integrate(inputs[i].source->getPose(), inputs[i].gpu, inputs[i].params, nullptr);
			}
		} else {
			active = 1;
		}

		frameCount++;

		stream->poll();
		display->update();
		//sleep_for(milliseconds(10));
	}
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "reconstruction_default"));
}
