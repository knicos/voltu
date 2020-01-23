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

	net->start();
	net->waitConnections();

	/*auto available = net.findAll<string>("list_streams");
	for (auto &a : available) {
		std::cout << " -- " << a << std::endl;
	}*/

	ftl::timer::start();

	try {
		nanogui::init();

		{
			nanogui::ref<ftl::gui::Screen> app = new ftl::gui::Screen(root, net, controller);
			app->drawAll();
			app->setVisible(true);
			nanogui::mainloop();

			LOG(INFO) << "Stopping...";
			ftl::timer::stop(false);
			ftl::pool.stop(true);
			LOG(INFO) << "All threads stopped.";
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
