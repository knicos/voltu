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
#include <ftl/rgbd/virtual.hpp>
#include <ftl/rgbd/streamer.hpp>
#include <ftl/slave.hpp>
#include <ftl/rgbd/group.hpp>
#include <ftl/threads.hpp>

#include "ilw.hpp"
#include <ftl/render/splat_render.hpp>

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

static Eigen::Affine3d create_rotation_matrix(float ax, float ay, float az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * rx * ry;
}

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

	// Create scene transform, intended for axis aligning the walls and floor
	Eigen::Matrix4d transform;
	if (root->getConfig()["transform"].is_object()) {
		auto &c = root->getConfig()["transform"];
		float rx = c.value("pitch", 0.0f);
		float ry = c.value("yaw", 0.0f);
		float rz = c.value("roll", 0.0f);
		float x = c.value("x", 0.0f);
		float y = c.value("y", 0.0f);
		float z = c.value("z", 0.0f);

		Eigen::Affine3d r = create_rotation_matrix(rx, ry, rz);
		Eigen::Translation3d trans(Eigen::Vector3d(x,y,z));
		Eigen::Affine3d t(trans);
		transform = t.matrix() * r.matrix();
		LOG(INFO) << "Set transform: " << transform;
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
				input->setPose(transform * input->getPose());
				continue;
			}
			input->setPose(transform * T->second);
		}
	}

	ftl::rgbd::FrameSet scene_A;  // Output of align process
	ftl::rgbd::FrameSet scene_B;  // Input of render process

	//ftl::voxhash::SceneRep *scene = ftl::create<ftl::voxhash::SceneRep>(root, "voxelhash");
	ftl::rgbd::Streamer *stream = ftl::create<ftl::rgbd::Streamer>(root, "stream", net);
	ftl::rgbd::VirtualSource *virt = ftl::create<ftl::rgbd::VirtualSource>(root, "virtual");
	ftl::render::Splatter *splat = ftl::create<ftl::render::Splatter>(root, "renderer", &scene_B);
	ftl::rgbd::Group group;
	ftl::ILW *align = ftl::create<ftl::ILW>(root, "merge");

	// Generate virtual camera render when requested by streamer
	virt->onRender([splat,virt,&scene_B](ftl::rgbd::Frame &out) {
		virt->setTimestamp(scene_B.timestamp);
		splat->render(virt, out);
	});
	stream->add(virt);

	for (size_t i=0; i<sources.size(); i++) {
		Source *in = sources[i];
		in->setChannel(Channel::Depth);
		group.addSource(in);
	}

	stream->setLatency(4);  // FIXME: This depends on source!?
	stream->run();

	bool busy = false;

	group.setLatency(4);
	group.setName("ReconGroup");
	group.sync([splat,virt,&busy,&slave,&scene_A,&scene_B,&align](ftl::rgbd::FrameSet &fs) -> bool {
		//cudaSetDevice(scene->getCUDADevice());

		if (slave.isPaused()) return true;
		
		if (busy) {
			LOG(INFO) << "Group frameset dropped: " << fs.timestamp;
			return true;
		}
		busy = true;

		// Swap the entire frameset to allow rapid return
		fs.swapTo(scene_A);

		ftl::pool.push([&scene_B,&scene_A,&busy,&slave,&align](int id) {
			//cudaSetDevice(scene->getCUDADevice());
			// TODO: Release frameset here...
			//cudaSafeCall(cudaStreamSynchronize(scene->getIntegrationStream()));

			UNIQUE_LOCK(scene_A.mtx, lk);

			// Send all frames to GPU, block until done?
			scene_A.upload(Channel::Colour + Channel::Depth);  // TODO: (Nick) Add scene stream.
			//align->process(scene_A);

			// TODO: To use second GPU, could do a download, swap, device change,
			// then upload to other device. Or some direct device-2-device copy.
			scene_A.swapTo(scene_B);
			LOG(INFO) << "Align complete... " << scene_A.timestamp;
			busy = false;
		});
		return true;
	});

	ftl::timer::stop();
	slave.stop();
	net->shutdown();
	delete align;
	delete splat;
	delete stream;
	delete virt;
	delete net;
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "reconstruction_default"));
}
