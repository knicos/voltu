#include <loguru.hpp>
#include <nlohmann/json.hpp>
#include <ftl/codecs/shapes.hpp>
#include <ftl/streams/filestream.hpp>

#include "inputoutput.hpp"

using ftl::gui2::InputOutput;

using ftl::codecs::Channel;

InputOutput::InputOutput(ftl::Configurable *root, ftl::net::Universe *net) :
		net_(net) {

	master_ = std::unique_ptr<ftl::ctrl::Master>(new ftl::ctrl::Master(root, net));
	master_->onLog([](const ftl::ctrl::LogEvent &e){
		const int v = e.verbosity;
		switch (v) {
		case -2:	LOG(ERROR) << "Remote log: " << e.message; break;
		case -1:	LOG(WARNING) << "Remote log: " << e.message; break;
		case 0:		LOG(INFO) << "Remote log: " << e.message; break;
		}
	});

	//net_->onConnect([this](ftl::net::Peer *p) {
		//ftl::pool.push([this](int id) {
			// FIXME: Find better option that waiting here.
			// Wait to make sure streams have started properly.
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));

			//_updateCameras(screen_->net()->findAll<string>("list_streams"));
		//});
	//});

	feed_ = std::unique_ptr<ftl::stream::Feed>
		(ftl::create<ftl::stream::Feed>(root, "feed", net));

	speaker_ = feed_->speaker();

	//auto* f = feed_->filter({ftl::codecs::Channel::Colour, ftl::codecs::Channel::Depth});
	//feed_->render(f, Eigen::Matrix4d::Identity());
}
