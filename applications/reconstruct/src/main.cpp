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
#include <ftl/rgbd/streamer.hpp>
#include <ftl/slave.hpp>

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
using ftl::rgbd::Source;
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
	Source *source;
	DepthCameraData gpu;
	DepthCameraParams params;
};

static void run(ftl::Configurable *root) {
	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Slave slave(net, root);
	
	net->start();
	net->waitConnections();
	
	std::vector<Cameras> inputs;
	auto sources = ftl::createArray<Source>(root, "sources", net); //root->get<vector<json_t>>("sources");

	if (sources.size() == 0) {
		LOG(ERROR) << "No sources configured!";
		return;
	}

	for (int i=0; i<sources.size(); i++) {
		Source *in = sources[i];
		auto &cam = inputs.emplace_back();
		cam.source = in;
		cam.params.m_imageWidth = 0;
	}

	// TODO	move this block in its own method and add automated tests
	//		for error cases
	
	std::optional<json_t> merge = root->get<json_t>("merge");
	if (!merge) {
		LOG(WARNING) << "Input merging not configured, using only first input in configuration";
		inputs = { inputs[0] };
		inputs[0].source->setPose(Eigen::Matrix4d::Identity());
	}

	if (inputs.size() > 1) {
		std::map<std::string, Eigen::Matrix4d> transformations;

		/*if ((*merge)["register"]) {
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
		else {*/
			if (loadTransformations(string(FTL_LOCAL_CONFIG_ROOT) + "/registration.json", transformations)) {
				LOG(INFO) << "Loaded camera trasformations from file";
			}
			else {
				LOG(ERROR) << "Error loading camera transformations from file";
			}
		//}

		for (auto &input : inputs) {
			string uri = input.source->getURI();
			auto T = transformations.find(uri);
			if (T == transformations.end()) {
				LOG(ERROR) << "Camera pose for " + uri + " not found in transformations";
				LOG(WARNING) << "Using only first configured source";
				// TODO: use target source if configured and found
				inputs = { inputs[0] };
				inputs[0].source->setPose(Eigen::Matrix4d::Identity());
				break;
			}
			input.source->setPose(T->second);
		}
	}

	// Displays and Inputs configured
	
	LOG(INFO) << "Using sources:";
	for (auto &input : inputs) { LOG(INFO) << "    " + (string) input.source->getURI(); }
	
	//ftl::rgbd::Display *display = ftl::create<ftl::rgbd::Display>(root, "display");
	ftl::rgbd::Source *virt = ftl::create<ftl::rgbd::Source>(root, "virtual", net);

	auto virtimpl = new ftl::rgbd::VirtualSource(virt);
	virt->customImplementation(virtimpl);

	ftl::voxhash::SceneRep *scene = ftl::create<ftl::voxhash::SceneRep>(root, "voxelhash");
	virtimpl->setScene(scene);
	//display->setSource(virt);

	ftl::rgbd::Streamer *stream = ftl::create<ftl::rgbd::Streamer>(root, "stream", net);
	stream->add(virt);
	// Also proxy all inputs
	for (auto &in : inputs) {
		stream->add(in.source);
	}

	unsigned char frameCount = 0;
	bool paused = false;

	// Keyboard camera controls
	//display->onKey([&paused](int key) {
	//	if (key == 32) paused = !paused;
	//});

	int active = inputs.size();
	while (ftl::running) {
		if (active == 0) {
			LOG(INFO) << "Waiting for sources...";
			sleep_for(milliseconds(1000));
		}

		active = 0;

		if (!slave.isPaused()) {
			//net.broadcast("grab");  // To sync cameras
			scene->nextFrame();
		
			// TODO(Nick) Improve sync further...
			for (size_t i = 0; i < inputs.size(); i++) {
				inputs[i].source->grab();
			}

			stream->wait();

			for (size_t i = 0; i < inputs.size(); i++) {
				if (!inputs[i].source->isReady()) {
					inputs[i].params.m_imageWidth = 0;
					// TODO(Nick) : Free gpu allocs if was ready before
					continue;
				} else {
					auto &cam = inputs[i];
					auto in = inputs[i].source;
					if (cam.params.m_imageWidth == 0) {
						cam.params.fx = in->parameters().fx;
						cam.params.fy = in->parameters().fy;
						cam.params.mx = -in->parameters().cx;
						cam.params.my = -in->parameters().cy;
						cam.params.m_imageWidth = in->parameters().width;
						cam.params.m_imageHeight = in->parameters().height;
						cam.params.m_sensorDepthWorldMax = in->parameters().maxDepth;
						cam.params.m_sensorDepthWorldMin = in->parameters().minDepth;
						cam.gpu.alloc(cam.params);
					}
					
					//LOG(INFO) << in->getURI() << " loaded " << cam.params.fx;
				}

				// Get the RGB-Depth frame from input
				Source *input = inputs[i].source;
				Mat rgb, depth;
				input->getFrames(rgb,depth);
				
				active += 1;

				if (depth.cols == 0) continue;

				// Must be in RGBA for GPU
				Mat rgba;
				cv::cvtColor(rgb,rgba, cv::COLOR_BGR2BGRA);

				inputs[i].params.flags = frameCount;

				// Send to GPU and merge view into scene
				inputs[i].gpu.updateParams(inputs[i].params);
				inputs[i].gpu.updateData(depth, rgba);
				// TODO(Nick): Use double pose
				scene->integrate(inputs[i].source->getPose().cast<float>(), inputs[i].gpu, inputs[i].params, nullptr);
			}
		} else {
			active = 1;
		}

		frameCount++;

		stream->poll();
		//display->update();
		//sleep_for(milliseconds(10));
	}
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "reconstruction_default"));
}
