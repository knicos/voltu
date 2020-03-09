#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/rgbd.hpp>
#include <ftl/master.hpp>
#include <ftl/net_configurable.hpp>

#include <loguru.hpp>

#include "screen.hpp"

#include <cuda_gl_interop.h>


int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "gui_default");
	ftl::net::Universe *net = ftl::create<ftl::net::Universe>(root, "net");

	int cuda_device;
	cudaSafeCall(cudaGetDevice(&cuda_device));
	//cudaSafeCall(cudaGLSetGLDevice(cuda_device));

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
			//nanogui::mainloop(20);

			float last_draw_time = 0.0f;

			while (ftl::running) {
				if (!app->visible()) {
					ftl::running = false;
				} else if (glfwWindowShouldClose(app->glfwWindow())) {
					app->setVisible(false);
					ftl::running = false;
				} else {
					float now = (float)glfwGetTime();
					float delta = now - last_draw_time;

					// Generate poses and render and virtual frame here
					// at full FPS (25 without VR and 90 with VR currently)
					app->drawFast();

					// Only draw the GUI at 25fps
					if (delta >= 0.04f) {
						last_draw_time = now;
						app->drawAll();
					}
				}

				/* Wait for mouse/keyboard or empty refresh events */
				//glfwWaitEvents();
				glfwPollEvents();
			}

        	/* Process events once more */
        	glfwPollEvents();

			LOG(INFO) << "Stopping...";
			ftl::timer::stop(false);
			ftl::pool.stop(true);
			LOG(INFO) << "All threads stopped.";
		}

		nanogui::shutdown();
	} catch (const ftl::exception &e) {
		LOG(ERROR) << "Fatal error: " << e.what();
		LOG(ERROR) << e.trace();
	} catch (const std::runtime_error &e) {
		std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
		#if defined(_WIN32)
			MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
		#else
			LOG(ERROR) << error_msg;
		#endif
		return -1;
	}


	net->shutdown();	
	delete controller;
	delete net;
	delete root;

	return 0;
}
