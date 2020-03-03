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
#include <ftl/master.hpp>
#include <ftl/rgbd/group.hpp>
#include <ftl/threads.hpp>
#include <ftl/codecs/writer.hpp>
#include <ftl/codecs/reader.hpp>

#include "reconstruction.hpp"

#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

//#include <opencv2/opencv.hpp>
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

#include <ftl/codecs/h264.hpp>
#include <ftl/codecs/hevc.hpp>

#include <ftl/streams/filestream.hpp>
#include <ftl/streams/receiver.hpp>
#include <ftl/streams/sender.hpp>
#include <ftl/streams/netstream.hpp>

#include <ftl/audio/source.hpp>

#include <cuda_profiler_api.h>

#include <nlohmann/json.hpp>

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


/* Build a generator using a deprecated list of source objects. */
static ftl::rgbd::Generator *createSourceGenerator(const std::vector<ftl::rgbd::Source*> &srcs) {
	
	auto *grp = new ftl::rgbd::Group();
	for (auto s : srcs) {
		s->setChannel(Channel::Depth);
		grp->addSource(s);
	}
	return grp;
}

static ftl::rgbd::Generator *createFileGenerator(ftl::Configurable *root, const std::string &filename) {
	ftl::stream::File *stream = ftl::create<ftl::stream::File>(root, "player");
	stream->set("filename", filename);

	ftl::stream::Receiver *gen = ftl::create<ftl::stream::Receiver>(root, "receiver");
	gen->setStream(stream);

	stream->begin();
	stream->select(0, Channel::Colour + Channel::Depth);  // TODO: Choose these elsewhere
	return gen;
}

static void threadSetCUDADevice() {
	// Ensure all threads have correct cuda device
	std::atomic<int> ijobs = 0;
	for (int i=0; i<ftl::pool.size(); ++i) {
		ftl::pool.push([&ijobs](int id) {
			ftl::cuda::setDevice();
			++ijobs;
			while (ijobs < ftl::pool.size()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
		});
	}
	while (ijobs < ftl::pool.size()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

static void run(ftl::Configurable *root) {
	// Use other GPU if available.
	ftl::cuda::setDevice(ftl::cuda::deviceCount()-1);
	threadSetCUDADevice();
	ftl::timer::setClockSlave(false);  // don't sync clocks.
	

	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Master ctrl(root, net);

	net->start();
	net->waitConnections();

	vector<ftl::Reconstruction*> groups;

	ftl::codecs::Channels channels;

	ftl::stream::Sender *sender = ftl::create<ftl::stream::Sender>(root, "sender");
	ftl::stream::Net *outstream = ftl::create<ftl::stream::Net>(root, "stream", net);
	outstream->set("uri", "ftl://test.utu.fi");
	outstream->begin();
	sender->setStream(outstream);

	ftl::audio::Source *audioSrc = nullptr;

	std::vector<Source*> sources;
	// Create a vector of all input RGB-Depth sources
	if (root->getConfig()["sources"].size() > 0) {
		sources = ftl::createArray<Source>(root, "sources", net);
		auto *gen = createSourceGenerator(sources);
		auto reconstr = ftl::create<ftl::Reconstruction>(root, "0", "0");
		reconstr->setGenerator(gen);
		reconstr->onFrameSet([sender](ftl::rgbd::FrameSet &fs) {
			fs.id = 0;
			sender->post(fs);
			return true;
		});
		groups.push_back(reconstr);
	}

	// Check paths for FTL files to load.
	auto paths = (*root->get<nlohmann::json>("paths"));
	int i = groups.size();
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
			reader.read(reader.getStartTime()+100, [&max_stream,&channels](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
				max_stream = max(max_stream, spkt.streamID);
				if ((int)spkt.channel < 32) channels |= spkt.channel;
			});
			reader.end();

			LOG(INFO) << "Found " << (max_stream+1) << " sources in " << path;

			auto *gen = createFileGenerator(root, path);

			auto reconstr = ftl::create<ftl::Reconstruction>(root, std::string("recon")+std::to_string(i), std::to_string(i));
			reconstr->setGenerator(gen);
			reconstr->onFrameSet([sender,i](ftl::rgbd::FrameSet &fs) {
				fs.id = i;
				//fs.frames[0].create(Channel::Data, 44);
				sender->post(fs);
				return true;
			});
			groups.push_back(reconstr);
			++i;

			// TODO: Temporary reconstruction local audio source for testing
			audioSrc = ftl::create<ftl::audio::Source>(root, "audio_test");

			audioSrc->onFrameSet([sender](ftl::audio::FrameSet &fs) {
				sender->post(fs);
				return true;
			});
		} else {
			ftl::URI uri(path);
			if (uri.getScheme() == ftl::URI::SCHEME_TCP || uri.getScheme() == ftl::URI::SCHEME_WS) {
				net->connect(path)->waitConnection();
			}
		}
	}

	if (groups.size() == 0) {
		LOG(ERROR) << "No sources configured!";

		auto stream_uris = net->findAll<std::string>("list_streams");

		if (stream_uris.size() > 0) {
			ftl::stream::Muxer *stream = ftl::create<ftl::stream::Muxer>(root, "muxstream");
			ftl::stream::Receiver *gen = ftl::create<ftl::stream::Receiver>(root, "receiver");
			gen->setStream(stream);

			sender->onRequest([stream](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
				if (spkt.channel == Channel::Colour) {
					stream->reset();
				}
			});

			int count = 0;
			for (auto &s : stream_uris) {
				LOG(INFO) << " --- found stream: " << s;
				auto *nstream = ftl::create<ftl::stream::Net>(stream, std::to_string(count), net);
				nstream->set("uri", s);
				stream->add(nstream);
				++count;
			}

			auto reconstr = ftl::create<ftl::Reconstruction>(root, std::string("recon")+std::to_string(i), std::to_string(i));
			//reconstr->setGenerator(gen);
			gen->onFrameSet([stream, reconstr](ftl::rgbd::FrameSet &fs) {
				stream->select(fs.id, Channel::Colour + Channel::Depth);
				return reconstr->post(fs);
			});

			gen->onAudio([sender](ftl::audio::FrameSet &fs) {
				sender->post(fs);
				return true;
			});

			int i = groups.size();
			reconstr->onFrameSet([sender,i](ftl::rgbd::FrameSet &fs) {
				fs.id = i;
				sender->post(fs);
				return true;
			});
			groups.push_back(reconstr);
			stream->begin();
		} else {
			return;
		}
	}

	LOG(INFO) << "Start timer";
	ftl::timer::start(true);

	if (audioSrc) delete audioSrc;

	LOG(INFO) << "Shutting down...";
	ftl::timer::stop();
	ftl::pool.stop(true);
	ctrl.stop();
	net->shutdown();

	//cudaProfilerStop();

	LOG(INFO) << "Deleting...";

	delete sender;
	delete outstream;
	delete net;
	for (auto g : groups) {
		delete g;
	}

	ftl::config::cleanup();  // Remove any last configurable objects.
	LOG(INFO) << "Done.";
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "reconstruction_default"));
}
