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
#include <ftl/rgbd.hpp>
#include <ftl/virtual_source.hpp>
#include <ftl/rgbd/streamer.hpp>
#include <ftl/slave.hpp>
#include <ftl/rgbd/group.hpp>

#include "ilw.hpp"
#include "splat_render.hpp"

#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <ftl/net/universe.hpp>

#include <ftl/registration.hpp>

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

using ftl::net::Universe;
using std::string;
using std::vector;
using ftl::rgbd::Source;
using ftl::config::json_t;
using ftl::rgbd::Channel;

using json = nlohmann::json;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
//using std::mutex;
//using std::unique_lock;

//using cv::Mat;

using ftl::registration::loadTransformations;
using ftl::registration::saveTransformations;

static void run(ftl::Configurable *root) {
	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Slave slave(net, root);
	
	net->start();
	net->waitConnections();
	
	// Create a vector of all input RGB-Depth sources
	auto sources = ftl::createArray<Source>(root, "sources", net);

	if (sources.size() == 0) {
		LOG(ERROR) << "No sources configured!";
		return;
	}

	// Must find pose for each source...
	if (sources.size() > 1) {
		std::map<std::string, Eigen::Matrix4d> transformations;

		if (loadTransformations(string(FTL_LOCAL_CONFIG_ROOT) + "/registration.json", transformations)) {
			LOG(INFO) << "Loaded camera trasformations from file";
		}
		else {
			LOG(ERROR) << "Error loading camera transformations from file";
		}

		for (auto &input : sources) {
			string uri = input->getURI();
			auto T = transformations.find(uri);
			if (T == transformations.end()) {
				LOG(ERROR) << "Camera pose for " + uri + " not found in transformations";
				//LOG(WARNING) << "Using only first configured source";
				// TODO: use target source if configured and found
				//sources = { sources[0] };
				//sources[0]->setPose(Eigen::Matrix4d::Identity());
				//break;
				continue;
			}
			input->setPose(T->second);
		}
	}

	//ftl::voxhash::SceneRep *scene = ftl::create<ftl::voxhash::SceneRep>(root, "voxelhash");
	ftl::rgbd::Streamer *stream = ftl::create<ftl::rgbd::Streamer>(root, "stream", net);
	ftl::rgbd::Source *virt = ftl::create<ftl::rgbd::Source>(root, "virtual", net);
	ftl::render::Splatter *splat = new ftl::render::Splatter();
	ftl::rgbd::Group group;
	ftl::ILW align;

	//auto virtimpl = new ftl::rgbd::VirtualSource(virt);
	//virt->customImplementation(virtimpl);
	//virtimpl->setScene(scene);
	stream->add(virt);

	for (size_t i=0; i<sources.size(); i++) {
		Source *in = sources[i];
		in->setChannel(Channel::Depth);
		//stream->add(in);
		//scene->addSource(in);
		group.addSource(in);
	}

	stream->run();

	bool busy = false;

	ftl::rgbd::FrameSet scene;

	group.setName("ReconGroup");
	group.sync([splat,virt,&busy,&slave](ftl::rgbd::FrameSet &fs) -> bool {
		//cudaSetDevice(scene->getCUDADevice());
		
		if (busy) {
			LOG(INFO) << "Group frameset dropped: " << fs.timestamp;
			return true;
		}
		busy = true;
		//scene->nextFrame();

		// Send all frames to GPU, block until done?
		// TODO: Allow non-block and keep frameset locked until later
		if (!slave.isPaused()) {
			//scene->upload(fs);
			for (auto &f : fs.frames) {
				f.upload(Channel::Colour + Channel::Depth); // TODO: (Nick) Add scene stream
			}
		}

		//int64_t ts = fs.timestamp;

		ftl::pool.push([splat,virt,&busy,&fs,&slave](int id) {
			//cudaSetDevice(scene->getCUDADevice());
			// TODO: Release frameset here...
			//cudaSafeCall(cudaStreamSynchronize(scene->getIntegrationStream()));

			if (!slave.isPaused()) {
				//scene->integrate();
				//scene->garbage();
				align.process(fs, scene);
			}

			// Don't render here... but update timestamp.
			splat->render(scene, virt); //, scene->getIntegrationStream());
			busy = false;
		});
		return true;
	});


	/*int active = sources.size();
	while (ftl::running) {
		if (active == 0) {
			LOG(INFO) << "Waiting for sources...";
			sleep_for(milliseconds(1000));
		}

		active = 0;

		if (!slave.isPaused()) {
			// Mark voxels as cleared
			scene->nextFrame();
		
			// Grab, upload frames and allocate voxel blocks
			active = scene->upload();

			// Make sure previous virtual camera frame has finished rendering
			//stream->wait();
			cudaSafeCall(cudaStreamSynchronize(scene->getIntegrationStream()));


			// Merge new frames into the voxel structure
			scene->integrate();

			//LOG(INFO) << "Allocated: " << scene->getOccupiedCount();

			// Remove any redundant voxels
			scene->garbage();

		} else {
			active = 1;
		}

		splat->render(virt, scene->getIntegrationStream());

		// Start virtual camera rendering and previous frame compression
		stream->poll();
	}*/
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "reconstruction_default"));
}
