#include "src_window.hpp"

#include "screen.hpp"
#include "camera.hpp"
#include "scene.hpp"

#include <nanogui/imageview.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/combobox.h>
#include <nanogui/label.h>
#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/layout.h>
#include <nanogui/vscrollpanel.h>

#include <ftl/streams/netstream.hpp>

#include "ftl/operators/colours.hpp"
#include "ftl/operators/segmentation.hpp"
#include "ftl/operators/mask.hpp"
#include "ftl/operators/antialiasing.hpp"

#ifdef HAVE_LIBARCHIVE
#include "ftl/rgbd/snapshot.hpp"
#endif

#include "thumbview.hpp"

using ftl::gui::SourceWindow;
using ftl::gui::Screen;
using ftl::gui::Scene;
using ftl::rgbd::Source;
using ftl::codecs::Channel;
using std::string;
using std::vector;
using ftl::config::json_t;

SourceWindow::SourceWindow(ftl::gui::Screen *screen)
		: nanogui::Window(screen, ""), screen_(screen) {
	setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 20, 5));

	using namespace nanogui;
	
	new Label(this, "Select Camera","sans-bold",20);

	auto vscroll = new VScrollPanel(this);
	ipanel_ = new Widget(vscroll);
	ipanel_->setLayout(new GridLayout(nanogui::Orientation::Horizontal, 2,
		nanogui::Alignment::Middle, 0, 5));

	screen->net()->onConnect([this](ftl::net::Peer *p) {
		UNIQUE_LOCK(mutex_, lk);
		_updateCameras(screen_->net()->findAll<string>("list_streams"));
	});

	UNIQUE_LOCK(mutex_, lk);
	stream_ = ftl::create<ftl::stream::Muxer>(screen->root(), "muxer");
	interceptor_ = ftl::create<ftl::stream::Intercept>(screen->root(), "intercept");
	interceptor_->setStream(stream_);
	receiver_ = ftl::create<ftl::stream::Receiver>(screen->root(), "receiver");
	receiver_->setStream(interceptor_);

	// Create a recorder
	recorder_ = ftl::create<ftl::stream::File>(screen->root(), "recorder");
	recorder_->setMode(ftl::stream::File::Mode::Write);

	interceptor_->onIntercept([this] (const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		//LOG(INFO) << (std::string)spkt;
		if (recorder_->active() && pkt.data.size() > 0) recorder_->post(spkt, pkt);
	});

	pre_pipeline_ = ftl::config::create<ftl::operators::Graph>(screen->root(), "pre_filters");
	//pre_pipeline_->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
	pre_pipeline_->append<ftl::operators::CrossSupport>("cross");
	pre_pipeline_->append<ftl::operators::DiscontinuityMask>("discontinuity");
	pre_pipeline_->append<ftl::operators::CullDiscontinuity>("remove_discontinuity");
	pre_pipeline_->append<ftl::operators::VisCrossSupport>("viscross")->set("enabled", false);

	cycle_ = 0;
	receiver_->onFrameSet([this](ftl::rgbd::FrameSet &fs) {
		fs.swapTo(frameset_);

		// Request the channels required by current camera configuration
		interceptor_->select(frameset_.id, _aggregateChannels());

		/*if (fs.frames[0].hasChannel(Channel::Data)) {
			int data = 0;
			fs.frames[0].get(Channel::Data, data);
			LOG(INFO) << "GOT DATA : " << data;
		}*/

		const auto *cstream = interceptor_;
		_createDefaultCameras(frameset_, cstream->available(fs.id).has(Channel::Depth));

		//LOG(INFO) << "Channels = " << (unsigned int)cstream->available(fs.id);

		// Enforce interpolated colour
		for (int i=0; i<frameset_.frames.size(); ++i) {
			frameset_.frames[i].createTexture<uchar4>(Channel::Colour, true);
		}

		pre_pipeline_->apply(frameset_, frameset_, 0);

		int i=0;
		for (auto cam : cameras_) {
			// Only update the camera periodically unless the active camera
			if (screen_->activeCamera() == cam.second.camera ||
				(screen_->activeCamera() == nullptr && cycle_ % cameras_.size() == i++))  cam.second.camera->update(frameset_);

			cam.second.camera->update(cstream->available(frameset_.id));
		}
		++cycle_;

		return true;
	});

	speaker_ = ftl::create<ftl::audio::Speaker>(screen_->root(), "speaker_test");

	receiver_->onAudio([this](ftl::audio::FrameSet &fs) {
		//LOG(INFO) << "Audio delay required = " << (ts - frameset_.timestamp) << "ms";
		speaker_->setDelay(fs.timestamp - frameset_.timestamp + ftl::timer::getInterval());  // Add Xms for local render time
		speaker_->queue(fs.timestamp, fs.frames[0]);
		return true;
	});

	ftl::timer::add(ftl::timer::kTimerMain, [this](int64_t ts) {
		auto *c = screen_->activeCamera();
		// Only offer full framerate render on active camera.
		if (c) {
			UNIQUE_LOCK(frameset_.mtx,lk);
			c->draw(frameset_);
		}
		return true;
	});

	_updateCameras(screen_->control()->getNet()->findAll<string>("list_streams"));

	// Also check for a file on command line.
	// Check paths for FTL files to load.
	auto paths = (*screen->root()->get<nlohmann::json>("paths"));

	for (auto &x : paths.items()) {
		std::string path = x.value().get<std::string>();
		auto eix = path.find_last_of('.');
		auto ext = path.substr(eix+1);

		// Command line path is ftl file
		if (ext == "ftl") {
			LOG(INFO) << "Found FTL file: " << path;
			auto *fstream = ftl::create<ftl::stream::File>(screen->root(), "ftlfile");
			fstream->set("filename", path);
			stream_->add(fstream);
		}
	}

	stream_->begin();
}

void SourceWindow::recordVideo(const std::string &filename) {
	if (!recorder_->active()) {
		recorder_->set("filename", filename);
		recorder_->begin();
		LOG(INFO) << "Recording started: " << filename;

		// TODO: Inject pose and calibrations
	}
}

void SourceWindow::stopRecordingVideo() {
	if (recorder_->active()) {
		recorder_->end();
		LOG(INFO) << "Recording stopped.";
	}
}

ftl::codecs::Channels<0> SourceWindow::_aggregateChannels() {
	ftl::codecs::Channels<0> cs = ftl::codecs::Channels<0>(Channel::Colour);
	for (auto cam : cameras_) {
		if (cam.second.camera->isVirtual()) {
			cs += Channel::Depth;
		} else {
			if (cam.second.camera->getChannel() != Channel::None) {
				cs += cam.second.camera->getChannel();
			}
		}
	}
	return cs;
}

void SourceWindow::_createDefaultCameras(ftl::rgbd::FrameSet &fs, bool makevirtual) {
	for (int i=0; i<fs.frames.size(); ++i) {
		int id = i;  // TODO: Include frameset id
		if (cameras_.find(id) == cameras_.end()) {
			auto *cam = new ftl::gui::Camera(screen_, 0, i);
			cameras_[id] = {
				cam,
				nullptr
			};
		}
	}

	if (makevirtual && cameras_.find(-1) == cameras_.end()) {
		auto *cam = new ftl::gui::Camera(screen_, 0, 255);
		cameras_[-1] = {
			cam,
			nullptr
		};
	}
}

std::vector<ftl::gui::Camera*> SourceWindow::getCameras() {
	auto cameras = std::vector<ftl::gui::Camera*>();
	cameras.reserve(cameras_.size());

	for (auto cam : cameras_) {
		cameras.push_back(cam.second.camera);
	}
	return cameras;
}

void SourceWindow::_updateCameras(const vector<string> &netcams) {
	if (netcams.size() == 0) return;

	for (auto s : netcams) {
		// FIXME: Check for already existing...
		//if (streams_.find(s) == cameras_.end()) {
			available_.push_back(s);
			json_t srcjson;
			srcjson["uri"] = s;
			screen_->root()->getConfig()["streams"].push_back(srcjson);
			//screen_->root()->getConfig()["receivers"].push_back(json_t{});
		//}
	}

	std::vector<ftl::stream::Net*> strms = ftl::createArray<ftl::stream::Net>(screen_->root(), "streams", screen_->net());

	for (int i=0; i<strms.size(); ++i) {
		auto *stream = strms[i];
		stream_->add(stream);

		LOG(INFO) << "Add Stream: " << stream->value("uri", std::string("NONE"));

		//Scene *scene = new Scene(receiver);
		//scenes_.push_back(scene);

		/*if (.find(src->getURI()) == cameras_.end()) {
			LOG(INFO) << "Making camera: " << src->getURI();

			// TODO: Need to have GUI wrapper for an entire stream... which
			// manages a set of cameras.

			auto *cam = new ftl::gui::Camera(screen_, src);
			cameras_[src->getURI()] = cam;
		} else {
			//LOG(INFO) << "Camera already exists: " << s;
		}*/
	}

	//refresh_thumbs_ = true;
	//if (thumbs_.size() != available_.size()) {
	//	thumbs_.resize(available_.size());
	//}
}

SourceWindow::~SourceWindow() {

}

void SourceWindow::draw(NVGcontext *ctx) {
	//if (refresh_thumbs_) {
		UNIQUE_LOCK(mutex_, lk);
		//refresh_thumbs_ = false;

		if (thumbs_.size() < cameras_.size()) thumbs_.resize(cameras_.size());

		//for (size_t i=0; i<thumbs_.size(); ++i) {
		int i = 0;
		for (auto &camera : cameras_) {
			cv::Mat t;
			auto *cam = camera.second.camera;
			if (cam) {
				if (cam->thumbnail(t)) {
					thumbs_[i].update(t);
				}
			}

			if (!camera.second.thumbview) camera.second.thumbview = new ftl::gui::ThumbView(ipanel_, screen_, cam);

			/*if ((size_t)ipanel_->childCount() < i+1) {
				new ftl::gui::ThumbView(ipanel_, screen_, cam);
			}*/
			if (thumbs_[i].isValid()) dynamic_cast<nanogui::ImageView*>(camera.second.thumbview)->bindImage(thumbs_[i].texture());
			++i;
		}

		// TODO(Nick) remove excess image views

		center();
	//}

	nanogui::Window::draw(ctx);
}
