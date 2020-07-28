/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#define LOGURU_WITH_STREAMS 1
#include <loguru.hpp>
#include <ftl/config.h>
#include <ftl/configuration.hpp>
#include <ftl/master.hpp>
#include <ftl/threads.hpp>
#include <ftl/codecs/channels.hpp>
#include <ftl/codecs/depth_convert_cuda.hpp>
#include <ftl/data/framepool.hpp>
#include <ftl/audio/speaker.hpp>

#include <nlohmann/json.hpp>

#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/quality/qualitypsnr.hpp>
#include <ftl/net/universe.hpp>

#include <ftl/streams/filestream.hpp>
#include <ftl/streams/receiver.hpp>
#include <ftl/streams/sender.hpp>
#include <ftl/streams/netstream.hpp>

#include <ftl/operators/colours.hpp>
#include <ftl/operators/mask.hpp>
#include <ftl/operators/segmentation.hpp>
#include <ftl/operators/depth.hpp>

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

using ftl::net::Universe;
using std::string;
using std::vector;
using ftl::config::json_t;
using ftl::codecs::Channel;
using ftl::codecs::codec_t;
using ftl::codecs::definition_t;

using json = nlohmann::json;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

static ftl::data::Generator *createFileGenerator(ftl::Configurable *root, ftl::data::Pool *pool, const std::string &filename) {
	ftl::stream::File *stream = ftl::create<ftl::stream::File>(root, "player");
	stream->set("filename", filename);

	ftl::stream::Receiver *gen = ftl::create<ftl::stream::Receiver>(root, "receiver", pool);
	gen->setStream(stream);

	stream->begin();
	stream->select(0, Channel::Colour + Channel::Depth);  // TODO: Choose these elsewhere
	return gen;
}

static void visualizeDepthMap(	const cv::Mat &depth, cv::Mat &out,
								const float max_depth)
{
	DCHECK(max_depth > 0.0);

	depth.convertTo(out, CV_8U, 255.0f / max_depth);
	out = 255 - out;
	//cv::Mat mask = (depth >= max_depth); // TODO (mask for invalid pixels)
	
	applyColorMap(out, out, cv::COLORMAP_JET);
	//out.setTo(cv::Scalar(0), mask);
	//cv::cvtColor(out,out, cv::COLOR_BGR2BGRA);
}

static void run(ftl::Configurable *root) {
	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Master ctrl(root, net);

	net->start();
	net->waitConnections();

	std::list<ftl::Handle> handles;
	ftl::data::Pool pool(2,10);

	std::list<ftl::data::Generator*> generators;

	// Check paths for FTL files to load.
	auto paths = (*root->get<nlohmann::json>("paths"));
	int i = 0; //groups.size();
	for (auto &x : paths.items()) {
		std::string path = x.value().get<std::string>();
		auto eix = path.find_last_of('.');
		auto ext = path.substr(eix+1);

		// Command line path is ftl file
		if (ext == "ftl") {
			auto *gen = createFileGenerator(root, &pool, path);
			generators.push_back(gen);
			++i;
		} else {
			ftl::URI uri(path);
			if (uri.getScheme() == ftl::URI::SCHEME_TCP || uri.getScheme() == ftl::URI::SCHEME_WS) {
				net->connect(path)->waitConnection();
			}
		}
	}

	auto stream_uris = net->findAll<std::string>("list_streams");

	if (stream_uris.size() > 0) {
		ftl::stream::Muxer *stream = ftl::create<ftl::stream::Muxer>(root, "muxstream");
		ftl::stream::Receiver *gen = ftl::create<ftl::stream::Receiver>(root, "receiver", &pool);
		ftl::stream::Sender *sender = ftl::create<ftl::stream::Sender>(root, "sender");
		gen->setStream(stream);
		sender->setStream(stream);

		int count = 0;
		for (auto &s : stream_uris) {
			LOG(INFO) << " --- found stream: " << s;
			auto *nstream = ftl::create<ftl::stream::Net>(stream, std::string("netstream")+std::to_string(count), net);
			nstream->set("uri", s);
			//nstream->select(0, {Channel::Colour}, true);
			stream->add(nstream);
			++count;
		}

		generators.push_back(gen);
		stream->begin();
		stream->select(0, Channel::Colour + Channel::Depth + Channel::AudioStereo, true);

		handles.push_back(std::move(pool.onFlush([sender](ftl::data::Frame &f, ftl::codecs::Channel c) {
			// Send only reponse channels on a per frame basis
			if (f.mode() == ftl::data::FrameMode::RESPONSE) {
				sender->post(f, c);
			}
			return true;
		})));
	}

	ftl::audio::Speaker *speaker = ftl::create<ftl::audio::Speaker>(root, "speaker");

	for (auto *g : generators) {
		handles.push_back(std::move(g->onFrameSet([&](std::shared_ptr<ftl::data::FrameSet> fs) {	
			LOG(INFO) << "Got frameset: " << fs->timestamp();
			for (auto &f : fs->frames) {
				if (f.has(Channel::Colour)) {
					cv::Mat tmp;
					f.get<cv::cuda::GpuMat>(Channel::Colour).download(tmp);
					cv::imshow(std::string("Frame")+std::to_string(f.id().id), tmp);
				}

				if (f.has(Channel::Depth)) {
					cv::Mat tmp;
					f.get<cv::cuda::GpuMat>(Channel::Depth).download(tmp);
					visualizeDepthMap(tmp,tmp,8.0f);
					cv::imshow(std::string("Depth")+std::to_string(f.id().id), tmp);
				}

				if (f.has(Channel::AudioStereo)) {
					const auto &audio = f.get<std::list<ftl::audio::Audio>>(Channel::AudioStereo).front();
					LOG(INFO) << "Got stereo: " << audio.data().size();
					if (f.source() == 0) {
						speaker->queue(f.timestamp(), f);
					}
				}
			}

			int k = cv::waitKey(10);

			// Send the key back to vision node (TESTING)
			if (k >= 0) {
				auto rf = fs->firstFrame().response();
				rf.create<int>(Channel::Control) = k;
			}

			return true;
		})));
	}

	LOG(INFO) << "Start timer";
	ftl::timer::start(true);

	LOG(INFO) << "Shutting down...";
	ftl::timer::stop();
	ftl::pool.stop(true);
	ctrl.stop();
	net->shutdown();

	//cudaProfilerStop();

	LOG(INFO) << "Deleting...";

	delete net;

	ftl::config::cleanup();  // Remove any last configurable objects.
	LOG(INFO) << "Done.";
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "tools_default"));
}
