#include <ftl/configuration.hpp>
#include <ftl/net.hpp>
#include <ftl/master.hpp>
#include <nlohmann/json.hpp>
#include <loguru.hpp>

#include <ftl/streams/filestream.hpp>
#include <ftl/streams/netstream.hpp>

#include <unordered_set>

using ftl::net::Universe;
using ftl::codecs::Channel;
using std::vector;
using std::string;


static std::atomic_int src_count = 0;


static void run(ftl::Configurable *root) {

	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Master ctrl(root, net);

	ftl::stream::Muxer *mux_in = ftl::create<ftl::stream::Muxer>(root, "muxer");
	ftl::stream::File *file_out = ftl::create<ftl::stream::File>(root, "output");

	std::unordered_set<ftl::codecs::Channel> channels;
	channels.insert(Channel::Colour);

	if (root->value("depth", false)) channels.insert(Channel::Depth);
	if (root->value("right", false)) channels.insert(Channel::Right);
	if (root->value("audio", false)) channels.insert(Channel::Audio);

	file_out->set("filename", root->value("filename", std::string("out.ftl")));
	file_out->setMode(ftl::stream::File::Mode::Write);
	file_out->begin();

	auto h1 = mux_in->onPacket([file_out](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		file_out->post(spkt, pkt);
		return true;
	});

	mux_in->begin();

	net->onConnect([mux_in,net,root,&channels](ftl::net::Peer *p) {
		ftl::pool.push([mux_in,root,net,p,&channels](int id) {
			try {
				auto peerstreams = p->call<std::vector<std::string>>("list_streams");

				for (const auto &s : peerstreams) {
					int fsid = src_count++;

					auto *ns = ftl::create<ftl::stream::Net>(root, std::string("input") + std::to_string(fsid), net);
					ns->set("uri", s);
					mux_in->add(ns, fsid);
					mux_in->begin();
					mux_in->select(fsid, channels, true);

					LOG(INFO) << "Recording: " << s;
				}
			} catch (...) {

			}
		});
	});

	if (net->isBound("add_stream")) net->unbind("add_stream");
	net->bind("add_stream", [mux_in,root,net,&channels](ftl::net::Peer &p, std::string uri){
		int fsid = src_count++;

		auto *ns = ftl::create<ftl::stream::Net>(root, std::string("input") + std::to_string(fsid), net);
		ns->set("uri", uri);
		mux_in->add(ns, fsid);
		mux_in->begin();
		mux_in->select(fsid, channels, true);

		LOG(INFO) << "Recording: " << uri;
	});

	net->start();
	net->waitConnections();

	// Add sources here
	if (root->getConfig().contains("sources")) {
		for (const auto &s : root->getConfig()["sources"]) {
			ftl::URI uri(s);
			auto scheme = uri.getScheme();
			if (scheme == ftl::URI::scheme_t::SCHEME_TCP || scheme == ftl::URI::scheme_t::SCHEME_WS) {
				net->connect(s);
			} else {
				LOG(ERROR) << "Unsupported URI: " << s;
			}
		}
	}

	// Add sources from command line as well
	auto paths = root->get<vector<string>>("paths");

	for (auto &x : *paths) {
		if (x != "") {
			ftl::URI uri(x);
			auto scheme = uri.getScheme();
			if (scheme == ftl::URI::scheme_t::SCHEME_TCP || scheme == ftl::URI::scheme_t::SCHEME_WS) {
				net->connect(x);
			} else {
				LOG(ERROR) << "Unsupported URI: " << x;
			}
		}
	}

	// Just do whatever jobs are available
	while (ftl::running) {
		auto f = ftl::pool.pop();
		if (f) {
			f(-1);
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	mux_in->end();
	file_out->end();
	delete mux_in;
	delete file_out;

	ftl::config::save();

	net->shutdown();
	LOG(INFO) << "Stopping...";
	//ftl::timer::stop(true);
	//LOG(INFO) << "Timer stopped...";
	ftl::pool.stop(true);
	LOG(INFO) << "All threads stopped.";

	delete net;
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "recorder_default"));

	// Save config changes and delete final objects
	ftl::config::cleanup();

	return ftl::exit_code;
}
