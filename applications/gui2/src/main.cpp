#include <memory>

#include <loguru.hpp>
#include <nlohmann/json.hpp>

#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/net_configurable.hpp>
#include <ftl/rgbd.hpp>

#include <nanogui/nanogui.h>

#include <cuda_gl_interop.h>

#include "inputoutput.hpp"
#include "module.hpp"
#include "screen.hpp"

#include "modules.hpp"

#ifdef HAVE_PYLON
#include <pylon/PylonIncludes.h>
#endif

using std::unique_ptr;
using std::make_unique;

/**
 * FTL Graphical User Interface
 * Single screen, loads configuration and sets up networking and input/output.
 * Loads required modules to gui.
 */
class FTLGui {
public:
	FTLGui(int argc, char **argv);
	~FTLGui();

	template<typename T>
	T* loadModule(const std::string &name);
	void mainloop();

private:
	std::unique_ptr<ftl::Configurable> root_;
	std::unique_ptr<ftl::net::Universe> net_;
	std::unique_ptr<ftl::gui2::InputOutput> io_;

	nanogui::ref<ftl::gui2::Screen> screen_;
};

template<typename T>
T* FTLGui::loadModule(const std::string &name) {
	return screen_->addModule<T>(name, root_.get(), screen_.get(), io_.get());
}

FTLGui::FTLGui(int argc, char **argv) {
	using namespace ftl::gui2;

	screen_ = new Screen();

	int cuda_device;
	cudaSafeCall(cudaGetDevice(&cuda_device));
	//cudaSafeCall(cudaGLSetGLDevice(cuda_device));

	root_ = unique_ptr<ftl::Configurable>(ftl::configure(argc, argv, "gui_default"));
	net_ = unique_ptr<ftl::net::Universe>(ftl::create<ftl::net::Universe>(root_.get(), "net"));
	io_ = make_unique<ftl::gui2::InputOutput>(root_.get(), net_.get());

	net_->start();
	net_->waitConnections();

	loadModule<Themes>("themes");
	loadModule<ThumbnailsController>("home")->activate();
	loadModule<Camera>("camera");
	loadModule<ConfigCtrl>("configwindow");
	loadModule<Statistics>("statistics");
#ifdef HAVE_CERES
	loadModule<Calibration>("calibration");
#endif
	auto *adder = loadModule<AddCtrl>("adder");

	for (int c = 1; c < argc; c++) {
		std::string path(argv[c]);
		try {
			io_->feed()->add(path);
			LOG(INFO) << "Add: " << path;
		}
		catch (const ftl::exception&) {
			LOG(ERROR) << "Could not add: " << path;
		}
	}

	if (io_->feed()->listSources().size() == 0) {
		adder->show();
	}

	net_->onDisconnect([this](ftl::net::Peer *p) {
		if (p->status() != ftl::net::Peer::kConnected) {
			screen_->showError("Connection Failed", std::string("Could not connect to network peer: ") + p->getURI());
		} else {
			screen_->showError("Disconnection", std::string("Network peer disconnected: ") + p->getURI());
		}
	});

	net_->onError([this](ftl::net::Peer *, const ftl::net::Error &err) {

	});
}

FTLGui::~FTLGui() {
	net_->shutdown();
}

void FTLGui::mainloop() {
	// implements similar main loop as nanogui::mainloop()

	ftl::timer::start();

	screen_->setVisible(true);
	screen_->drawAll();

	float last_draw_time = 0.0f;

	while (ftl::running) {
		if (!screen_->visible()) {
			ftl::running = false;
		}
		else if (glfwWindowShouldClose(screen_->glfwWindow())) {
			screen_->setVisible(false);
			ftl::running = false;
		}
		else {
			float now = float(glfwGetTime());
			float delta = now - last_draw_time;

			// Generate poses and render and virtual frame here
			// at full FPS (25 without VR and 90 with VR currently)
			//screen_->render();

			io_->feed()->render();

			// Only draw the GUI at 25fps
			if (delta >= 0.04f) {
				last_draw_time = now;
				screen_->drawAll();
			}
		}

		// Wait for mouse/keyboard or empty refresh events
		glfwWaitEventsTimeout(0.02); // VR headest issues
		//glfwPollEvents();
	}

	// Process events once more
	glfwPollEvents();

	// Stop everything before deleting feed etc
	LOG(INFO) << "Stopping...";
	ftl::timer::stop(true);
	ftl::pool.stop(true);
	LOG(INFO) << "All threads stopped.";
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
	#ifdef HAVE_PYLON
	Pylon::PylonAutoInitTerm autoInitTerm;
	#endif

	// Note: This causes 100% CPU use but prevents the random frame drops.
	ftl::timer::setHighPrecision(true);

	{
		nanogui::init();
		
		FTLGui gui(argc, argv);

		try {
			gui.mainloop();
		}
		catch (const ftl::exception &e) {
			#ifdef WIN32
				std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what()) + std::string("\r\n") + std::string(e.trace());
				MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
			#else
				LOG(ERROR) << "Fatal error: " << e.what();
				LOG(ERROR) << e.trace();
			#endif
		}
		catch (const std::runtime_error &e) {
			std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
			#ifdef WIN32
				MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
				LOG(ERROR) << error_msg;
			#else
				LOG(ERROR) << error_msg;
			#endif
			return -1;
		}
	}

	// Must be after ~FTLGui since it destroys GL context.
	nanogui::shutdown();

	// Save config changes and delete final objects
	ftl::config::cleanup();

	return 0;
}
