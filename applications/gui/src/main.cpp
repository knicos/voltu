#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/rgbd.hpp>
#include <ftl/master.hpp>
#include <ftl/net_configurable.hpp>

#include <loguru.hpp>

#include "screen.hpp"


int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "gui_default");
	ftl::net::Universe *net = ftl::create<ftl::net::Universe>(root, "net");

	ftl::ctrl::Master *controller = new ftl::ctrl::Master(root, net);
	controller->onLog([](const ftl::ctrl::LogEvent &e){
		const int v = e.verbosity;
		switch (v) {
		case -2:	LOG(ERROR) << "Remote log: " << e.message; break;
		case -1:	LOG(WARNING) << "Remote log: " << e.message; break;
		case 0:		LOG(INFO) << "Remote log: " << e.message; break;
		}
	});

	std::map<ftl::UUID, std::vector<ftl::NetConfigurable*>> peerConfigurables;

	net->onConnect([&controller, &peerConfigurables](ftl::net::Peer *p) {
		ftl::UUID peer = p->id();
		auto cs = controller->getConfigurables(peer);
		for (auto c : cs) {
			ftl::config::json_t *configuration = new ftl::config::json_t;
			*configuration = controller->get(peer, c);
			if (!configuration->empty()) {
				ftl::NetConfigurable *nc = new ftl::NetConfigurable(peer, c, *controller, *configuration);
				peerConfigurables[peer].push_back(nc);
			}
		}
	});

	net->onDisconnect([&peerConfigurables](ftl::net::Peer *p) {
		ftl::UUID peer = p->id();
		for (ftl::NetConfigurable *nc : peerConfigurables[peer]) {
			ftl::config::json_t *configuration = &(nc->getConfig());
			delete nc;
			delete configuration;
		}
	});

	net->start();
	net->waitConnections();

	/*auto available = net.findAll<string>("list_streams");
	for (auto &a : available) {
		std::cout << " -- " << a << std::endl;
	}*/

	try {
		nanogui::init();

		/* scoped variables */ {
			nanogui::ref<ftl::gui::Screen> app = new ftl::gui::Screen(root, net, controller);
			app->drawAll();
			app->setVisible(true);
			nanogui::mainloop();
		}

		nanogui::shutdown();
	} catch (const std::runtime_error &e) {
		std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
		#if defined(_WIN32)
			MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
		#else
			std::cerr << error_msg << std::endl;
		#endif
		return -1;
	}

	net->shutdown();
	delete controller;
	delete net;
	delete root;

	return 0;
}
