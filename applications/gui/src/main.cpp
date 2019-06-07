#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/rgbd.hpp>
#include <ftl/master.hpp>

#include <loguru.hpp>

#include <opencv2/opencv.hpp>

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/imageview.h>
#include <nanogui/combobox.h>
#include <nanogui/label.h>

#include "ctrl_window.hpp"
#include "src_window.hpp"

using std::string;
using ftl::rgbd::RGBDSource;

/*struct SourceViews {
	ftl::rgbd::RGBDSource *source;
	GLTexture texture;
	nanogui::ImageView *view;
};*/



class FTLApplication : public nanogui::Screen {
	public:
	explicit FTLApplication(ftl::Configurable *root, ftl::net::Universe *net, ftl::ctrl::Master *controller) : nanogui::Screen(Eigen::Vector2i(1024, 768), "FT-Lab GUI") {
		using namespace nanogui;
		net_ = net;

		auto cwindow = new ftl::gui::ControlWindow(this, controller);
		auto swindow = new ftl::gui::SourceWindow(this, controller);

		setVisible(true);
		performLayout();
	}

	virtual void draw(NVGcontext *ctx) {
		nvgText(ctx, 10, 10, "FT-Lab Remote Presence System", NULL);

		/* Draw the user interface */
		Screen::draw(ctx);
	}

	private:
	//std::vector<SourceViews> sources_;
	ftl::net::Universe *net_;
};

int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "gui_default");
	ftl::net::Universe *net = ftl::create<ftl::net::Universe>(root, "net");

	net->waitConnections();

	ftl::ctrl::Master controller(root, net);
	controller.onLog([](const ftl::ctrl::LogEvent &e){
		const int v = e.verbosity;
		switch (v) {
		case -2:	LOG(ERROR) << "Remote log: " << e.message; break;
		case -1:	LOG(WARNING) << "Remote log: " << e.message; break;
		case 0:		LOG(INFO) << "Remote log: " << e.message; break;
		}
	});

	/*auto available = net.findAll<string>("list_streams");
	for (auto &a : available) {
		std::cout << " -- " << a << std::endl;
	}*/

	try {
		nanogui::init();

		/* scoped variables */ {
			nanogui::ref<FTLApplication> app = new FTLApplication(root, net, &controller);
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

	delete net;
	delete root;

	return 0;
}