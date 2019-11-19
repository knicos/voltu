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
#include <ftl/codecs/writer.hpp>
#include <ftl/codecs/reader.hpp>

#include "ilw/ilw.hpp"
#include <ftl/render/tri_render.hpp>

#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <ftl/net/universe.hpp>

#include <ftl/operators/smoothing.hpp>
#include <ftl/operators/colours.hpp>
#include <ftl/operators/normals.hpp>
#include <ftl/operators/filling.hpp>
#include <ftl/operators/segmentation.hpp>
#include <ftl/operators/mask.hpp>
#include <ftl/operators/antialiasing.hpp>
#include <ftl/operators/mvmls.hpp>
#include <ftl/operators/clipping.hpp>

#include <ftl/cuda/normals.hpp>
#include <ftl/registration.hpp>

#include <cuda_profiler_api.h>

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

using ftl::net::Universe;
using std::string;
using std::vector;
using ftl::rgbd::Source;
using ftl::config::json_t;
using ftl::codecs::Channel;

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

// TODO:	*	Remove this class (requires more general solution). Also does not
//				process disconnections/reconnections/types etc. correctly.
//			*	Update when new options become available.

class ConfigProxy {
	private:
	vector<ftl::UUID> peers_;
	vector<std::string> uris_;
	ftl::net::Universe *net_;
	
	public:
	ConfigProxy(ftl::net::Universe *net) {
		net_ = net;

		auto response = net_->findAll<std::string>("node_details");
		for (auto &r : response) {
			auto r_json = json_t::parse(r);
			peers_.push_back(ftl::UUID(r_json["id"].get<std::string>()));
			uris_.push_back(r_json["title"].get<std::string>());
		}
	}

	void add(ftl::Configurable *root, const std::string &uri, const std::string &name) {
		auto config = json_t::parse(net_->call<string>(peers_[0], "get_cfg", uris_[0] + "/" + uri));
		auto *proxy = ftl::create<ftl::Configurable>(root, name);
		
		try {
			for (auto &itm : config.get<json::object_t>()) {
				auto key = itm.first;
				auto value = itm.second;
				if (*key.begin() == '$') { continue; }

				proxy->set(key, value);
				proxy->on(key, [this, uri, key, value, proxy](const ftl::config::Event&) {
					for (size_t i = 0; i < uris_.size(); i++) {
						// TODO: check that config exists?
						auto peer = peers_[i];
						std::string name = uris_[i] + "/" + uri + "/" + key;
						net_->send(peer, "update_cfg", name, proxy->getConfig()[key].dump());
					}
				});
			}
		}
		catch (nlohmann::detail::type_error) {
			LOG(ERROR) << "Failed to add config proxy for: " << uri << "/" << name;
		}
	}
};

static void run(ftl::Configurable *root) {
	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Slave slave(net, root);

	// Controls
	auto *controls = ftl::create<ftl::Configurable>(root, "controls");
	
	net->start();
	net->waitConnections();
	
	// Check paths for an FTL file to load...
	auto paths = (*root->get<nlohmann::json>("paths"));
	for (auto &x : paths.items()) {
		std::string path = x.value().get<std::string>();
		auto eix = path.find_last_of('.');
		auto ext = path.substr(eix+1);

		// Command line path is ftl file
		if (ext == "ftl") {
			// Create temp reader to count number of sources found in file
			std::ifstream file;
			file.open(path);
			ftl::codecs::Reader reader(file);
			reader.begin();

			int max_stream = 0;
			reader.read(reader.getStartTime()+100, [&max_stream](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
				max_stream = max(max_stream, spkt.streamID);
			});
			reader.end();

			LOG(INFO) << "Found " << (max_stream+1) << " sources in " << path;

			int N = root->value("N", 100);

			// For each stream found, add a source object
			for (int i=0; i<=min(max_stream,N-1); ++i) {
				root->getConfig()["sources"].push_back(nlohmann::json{{"uri",std::string("file://") + path + std::string("#") + std::to_string(i)}});
			}
		}
	}
	
	// Create a vector of all input RGB-Depth sources
	auto sources = ftl::createArray<Source>(root, "sources", net);

	if (sources.size() == 0) {
		LOG(ERROR) << "No sources configured!";
		return;
	}
	
	ConfigProxy *configproxy = nullptr;
	if (net->numberOfPeers() > 0) {
		configproxy = new ConfigProxy(net); // TODO delete
		auto *disparity = ftl::create<ftl::Configurable>(root, "disparity");
		configproxy->add(disparity, "source/disparity/algorithm", "algorithm");
		configproxy->add(disparity, "source/disparity/bilateral_filter", "bilateral_filter");
		configproxy->add(disparity, "source/disparity/optflow_filter", "optflow_filter");
		configproxy->add(disparity, "source/disparity/mls", "mls");
		configproxy->add(disparity, "source/disparity/cross", "cross");
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
	} else {
		transform.setIdentity();
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
	root->set("tags", nlohmann::json::array({ root->getID()+"/virtual" }));
	ftl::render::Triangular *splat = ftl::create<ftl::render::Triangular>(root, "renderer", &scene_B);
	ftl::rgbd::Group *group = new ftl::rgbd::Group;
	ftl::ILW *align = ftl::create<ftl::ILW>(root, "merge");

	int o = root->value("origin_pose", 0) % sources.size();
	virt->setPose(sources[o]->getPose());

	auto *renderpipe = ftl::config::create<ftl::operators::Graph>(root, "render_pipe");
	renderpipe->append<ftl::operators::ColourChannels>("colour");  // Generate interpolation texture...
	renderpipe->append<ftl::operators::FXAA>("antialiasing"); 

	// Generate virtual camera render when requested by streamer
	virt->onRender([splat,virt,&scene_B,align,renderpipe](ftl::rgbd::Frame &out) {
		//virt->setTimestamp(scene_B.timestamp);
		// Do we need to convert Lab to BGR?
		if (align->isLabColour()) {
			for (auto &f : scene_B.frames) {
				auto &col = f.get<cv::cuda::GpuMat>(Channel::Colour);
				cv::cuda::cvtColor(col,col, cv::COLOR_Lab2BGR); // TODO: Add stream
			}
		}
		splat->render(virt, out);
		renderpipe->apply(out, out, virt, 0);
	});
	stream->add(virt);

	for (size_t i=0; i<sources.size(); i++) {
		Source *in = sources[i];
		in->setChannel(Channel::Depth);
		group->addSource(in);
	}

	// ---- Recording code -----------------------------------------------------

	std::ofstream fileout;
	ftl::codecs::Writer writer(fileout);
	auto recorder = [&writer,&group](ftl::rgbd::Source *src, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		ftl::codecs::StreamPacket s = spkt;

		// Patch stream ID to match order in group
		s.streamID = group->streamID(src);
		writer.write(s, pkt);
	};

	root->set("record", false);

	// Allow stream recording
	root->on("record", [&group,&fileout,&writer,&recorder](const ftl::config::Event &e) {
		if (e.entity->value("record", false)) {
			char timestamp[18];
			std::time_t t=std::time(NULL);
			std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
			fileout.open(std::string(timestamp) + ".ftl");

			writer.begin();
			group->addRawCallback(std::function(recorder));

			// TODO: Write pose+calibration+config packets
			auto sources = group->sources();
			for (size_t i=0; i<sources.size(); ++i) {
				//writeSourceProperties(writer, i, sources[i]);
				sources[i]->inject(Channel::Calibration, sources[i]->parameters(), Channel::Left, sources[i]->getCapabilities());
				sources[i]->inject(sources[i]->getPose()); 
			}
		} else {
			group->removeRawCallback(recorder);
			writer.end();
			fileout.close();
		}
	});

	// -------------------------------------------------------------------------

	stream->setLatency(6);  // FIXME: This depends on source!?
	//stream->add(group);
	stream->run();

	bool busy = false;

	// Create the source depth map pipeline
	auto *pipeline1 = ftl::config::create<ftl::operators::Graph>(root, "pre_filters");
	pipeline1->append<ftl::operators::ClipScene>("clipping");
	pipeline1->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
	//pipeline1->append<ftl::operators::HFSmoother>("hfnoise");  // Remove high-frequency noise
	pipeline1->append<ftl::operators::Normals>("normals");  // Estimate surface normals
	//pipeline1->append<ftl::operators::SmoothChannel>("smoothing");  // Generate a smoothing channel
	//pipeline1->append<ftl::operators::ScanFieldFill>("filling");  // Generate a smoothing channel
	pipeline1->append<ftl::operators::CrossSupport>("cross");
	pipeline1->append<ftl::operators::DiscontinuityMask>("discontinuity");
	pipeline1->append<ftl::operators::CullDiscontinuity>("remove_discontinuity");
	//pipeline1->append<ftl::operators::AggreMLS>("mls");  // Perform MLS (using smoothing channel)
	pipeline1->append<ftl::operators::VisCrossSupport>("viscross")->set("enabled", false);
	pipeline1->append<ftl::operators::MultiViewMLS>("mvmls");
	// Alignment


	group->setLatency(4);
	group->setName("ReconGroup");
	group->sync([splat,virt,&busy,&slave,&scene_A,&scene_B,&align,controls,pipeline1](ftl::rgbd::FrameSet &fs) -> bool {
		//cudaSetDevice(scene->getCUDADevice());

		//if (slave.isPaused()) return true;
		if (controls->value("paused", false)) return true;
		
		if (busy) {
			LOG(INFO) << "Group frameset dropped: " << fs.timestamp;
			return true;
		}
		busy = true;

		// Swap the entire frameset to allow rapid return
		fs.swapTo(scene_A);

		ftl::pool.push([&scene_B,&scene_A,&busy,&slave,&align, pipeline1](int id) {
			//cudaSetDevice(scene->getCUDADevice());
			// TODO: Release frameset here...
			//cudaSafeCall(cudaStreamSynchronize(scene->getIntegrationStream()));

			UNIQUE_LOCK(scene_A.mtx, lk);

			pipeline1->apply(scene_A, scene_A, 0);
			align->process(scene_A);


			// TODO: To use second GPU, could do a download, swap, device change,
			// then upload to other device. Or some direct device-2-device copy.
			scene_A.swapTo(scene_B);
			LOG(INFO) << "Align complete... " << scene_A.timestamp;
			busy = false;
		});
		return true;
	});

	LOG(INFO) << "Start timer";
	ftl::timer::start(true);

	LOG(INFO) << "Shutting down...";
	ftl::timer::stop();
	slave.stop();
	net->shutdown();
	ftl::pool.stop();

	cudaProfilerStop();

	LOG(INFO) << "Deleting...";

	delete align;
	delete splat;
	delete stream;
	delete virt;
	delete net;
	delete group;

	ftl::config::cleanup();  // Remove any last configurable objects.
	LOG(INFO) << "Done.";
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "reconstruction_default"));
}
