#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/group.hpp>

int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "viewer_default");
	ftl::net::Universe *net = ftl::create<ftl::net::Universe>(root, "net");

	net->start();
	net->waitConnections();

	auto sources = ftl::createArray<ftl::rgbd::Source>(root, "sources", net);

	ftl::rgbd::Group group;
	for (auto s : sources) {
		s->setChannel(ftl::rgbd::kChanRight);
		group.addSource(s);
	}

	group.sync([](const ftl::rgbd::FrameSet &fs) {
		LOG(INFO) << "Complete set: " << fs.timestamp;
		return true;
	});

	while (ftl::running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		for (auto s : sources) s->grab(30);
	}

	return 0;
}
