#include <ftl/configuration.hpp>
#include <ftl/net.hpp>
#include <ftl/streams/feed.hpp>
#include <ftl/master.hpp>
#include <nlohmann/json.hpp>
#include <loguru.hpp>

#include "ftl/operators/smoothing.hpp"
#include "ftl/operators/colours.hpp"
#include "ftl/operators/normals.hpp"
#include "ftl/operators/filling.hpp"
#include "ftl/operators/segmentation.hpp"
#include "ftl/operators/mask.hpp"
#include "ftl/operators/antialiasing.hpp"
#include "ftl/operators/mvmls.hpp"
#include "ftl/operators/clipping.hpp"
#include <ftl/operators/disparity.hpp>
#include <ftl/operators/poser.hpp>
#include <ftl/operators/detectandtrack.hpp>

using ftl::net::Universe;
using ftl::stream::Feed;
using ftl::codecs::Channel;
using std::vector;
using std::string;

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
	ftl::timer::setClockSlave(false);
	ftl::timer::setHighPrecision(true);

	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Master ctrl(root, net);

	net->start();
	net->waitConnections();

	Feed *feed = ftl::create<Feed>(root, "feed", net);
	std::string group_name = root->value("group", std::string("Reconstruction"));

	feed->set("uri", root->value("uri", std::string("ftl://ftlab.utu.fi/reconstruction")));
	feed->setPipelineCreator([](ftl::operators::Graph *pipeline) {
		LOG(INFO) << "Using reconstruction pipeline creator";

		pipeline->append<ftl::operators::DepthChannel>("depth")->value("enabled", false);  // Ensure there is a depth channel
		pipeline->append<ftl::operators::DisparityBilateralFilter>("bilateral_filter")->value("enabled", false);
		pipeline->append<ftl::operators::DisparityToDepth>("calculate_depth")->value("enabled", false);
		pipeline->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
		pipeline->append<ftl::operators::ClipScene>("clipping")->value("enabled", false);
		pipeline->append<ftl::operators::DetectAndTrack>("facedetection")->value("enabled", false);
		pipeline->append<ftl::operators::ArUco>("aruco")->value("enabled", false);
		//pipeline_->append<ftl::operators::HFSmoother>("hfnoise");  // Remove high-frequency noise
		pipeline->append<ftl::operators::Normals>("normals");  // Estimate surface normals
		//pipeline_->append<ftl::operators::SmoothChannel>("smoothing");  // Generate a smoothing channel
		//pipeline_->append<ftl::operators::ScanFieldFill>("filling");  // Generate a smoothing channel
		pipeline->append<ftl::operators::CrossSupport>("cross");
		pipeline->append<ftl::operators::DiscontinuityMask>("discontinuity");
		pipeline->append<ftl::operators::CrossSupport>("cross2")->value("discon_support", true);
		pipeline->append<ftl::operators::BorderMask>("border_mask")->value("enabled", false);
		pipeline->append<ftl::operators::CullDiscontinuity>("remove_discontinuity")->set("enabled", false);
		//pipeline_->append<ftl::operators::AggreMLS>("mls");  // Perform MLS (using smoothing channel)
		pipeline->append<ftl::operators::VisCrossSupport>("viscross")->value("enabled", false);
		pipeline->append<ftl::operators::MultiViewMLS>("mvmls");
		pipeline->append<ftl::operators::Poser>("poser")->value("enabled", false);
	});

	// Add sources here
	if (root->getConfig().contains("sources")) {
		for (const auto &s : root->getConfig()["sources"]) {
			ftl::URI uri(s);
			uri.setAttribute("group", group_name);
			feed->add(uri);
		}
	}

	// Add sources from command line as well
	auto paths = root->get<vector<string>>("paths");
	string file = "";

	for (auto &x : *paths) {
		if (x != "") {
			ftl::URI uri(x);
			uri.setAttribute("group", group_name);
			feed->add(uri);
		}
	}

	// Automatically add any new sources
	auto nsrc_handle = feed->onNewSources([feed,group_name](const vector<string> &srcs) {
		for (const auto &s : srcs) {
			ftl::URI uri(s);
			uri.setAttribute("group", group_name);
			feed->add(uri);
		}
		return true;
	});

	auto *filter = feed->filter({Channel::Colour, Channel::Depth});
	
	//feed->lowLatencyMode();
	feed->startStreaming(filter);

	// Just do whatever jobs are available
	while (ftl::running) {
		auto f = ftl::pool.pop();
		if (f) {
			f(-1);
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	nsrc_handle.cancel();
	feed->stopRecording();
	feed->removeFilter(filter);

	net->shutdown();
	LOG(INFO) << "Stopping...";
	ftl::timer::stop(true);
	LOG(INFO) << "Timer stopped...";
	ftl::pool.stop(true);
	LOG(INFO) << "All threads stopped.";

	delete feed;
	delete net;
	delete root;
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "reconstruction_default"));

	// Save config changes and delete final objects
	ftl::config::cleanup();

	return ftl::exit_code;
}
