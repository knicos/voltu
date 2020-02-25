#include "src_window.hpp"

#include "screen.hpp"
#include "camera.hpp"
#include "scene.hpp"

#include <ftl/profiler.hpp>

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

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <ftl/streams/netstream.hpp>

#include "ftl/operators/colours.hpp"
#include "ftl/operators/segmentation.hpp"
#include "ftl/operators/mask.hpp"
#include "ftl/operators/antialiasing.hpp"
#include <ftl/operators/smoothing.hpp>
#include <ftl/operators/disparity.hpp>
#include <ftl/operators/detectandtrack.hpp>
#include <ftl/operators/weighting.hpp>
#include <ftl/operators/mvmls.hpp>

#include <nlohmann/json.hpp>

#ifdef HAVE_LIBARCHIVE
#include "ftl/rgbd/snapshot.hpp"
#endif

#include "thumbview.hpp"

using ftl::gui::SourceWindow;
using ftl::gui::Screen;
using ftl::gui::Scene;
using ftl::rgbd::Source;
using ftl::codecs::Channel;
using ftl::codecs::Channels;
using std::string;
using std::vector;
using ftl::config::json_t;

static ftl::rgbd::Generator *createSourceGenerator(ftl::Configurable *root, const std::vector<ftl::rgbd::Source*> &srcs) {
	
	auto *grp = new ftl::rgbd::Group();
	/*auto pipeline = ftl::config::create<ftl::operators::Graph>(root, "pipeline");
	pipeline->append<ftl::operators::DetectAndTrack>("facedetection")->value("enabled", false);
	pipeline->append<ftl::operators::ArUco>("aruco")->value("enabled", false);
	pipeline->append<ftl::operators::DepthChannel>("depth");  // Ensure there is a depth channel
	grp->addPipeline(pipeline);*/

	for (auto s : srcs) {
		s->setChannel(Channel::Depth);
		grp->addSource(s);
	}
	return grp;
}

SourceWindow::SourceWindow(ftl::gui::Screen *screen)
		: nanogui::Window(screen, ""), screen_(screen) {
	setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 20, 5));

	using namespace nanogui;
	
	new Label(this, "Select Camera","sans-bold",20);

	// FIXME: Reallocating the vector may currently causes thread issues since
	// it might be in use elsewhere. A safer mechanism is needed for sharing
	// framesets. Temporary solution: preallocate enough slots.
	pre_pipelines_.reserve(5);
	framesets_.reserve(5);

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

	paused_ = false;
	cycle_ = 0;
	receiver_->onFrameSet([this](ftl::rgbd::FrameSet &fs) {
		return _processFrameset(fs, true);
	});

	speaker_ = ftl::create<ftl::audio::Speaker>(screen_->root(), "speaker_test");

	receiver_->onAudio([this](ftl::audio::FrameSet &fs) {
		if (framesets_.size() == 0) return true;
		auto *c = screen_->activeCamera();
		int64_t renddelay = (c) ? c->getFrameTimeMS() : 0;
		speaker_->setDelay(fs.timestamp - framesets_[0]->timestamp + renddelay);  // Add Xms for local render time
		speaker_->queue(fs.timestamp, fs.frames[0]);
		return true;
	});

	/*ftl::timer::add(ftl::timer::kTimerMain, [this](int64_t ts) {
		auto *c = screen_->activeCamera();
		// Only offer full framerate render on active camera.
		if (c) {
			c->draw(framesets_);
		}
		return true;
	});*/

	// Add network sources
	_updateCameras(screen_->control()->getNet()->findAll<string>("list_streams"));

	// Also check for a file on command line.
	// Check paths for FTL files to load.
	auto paths = (*screen->root()->get<nlohmann::json>("paths"));

	int ftl_count = available_.size();
	for (auto &x : paths.items()) {
		std::string path = x.value().get<std::string>();
		auto eix = path.find_last_of('.');
		auto ext = path.substr(eix+1);

		// Command line path is ftl file
		if (ext == "ftl") {
			LOG(INFO) << "Found FTL file: " << path;
			auto *fstream = ftl::create<ftl::stream::File>(screen->root(), std::string("ftlfile-")+std::to_string(ftl_count+1));
			fstream->set("filename", path);
			stream_->add(fstream, ftl_count++);
		} else if (path.rfind("device:", 0) == 0) {
			ftl::URI uri(path);
			uri.to_json(screen->root()->getConfig()["sources"].emplace_back());
		}
	}

	// Finally, check for any device sources configured
	std::vector<Source*> devices;
	// Create a vector of all input RGB-Depth sources
	if (screen->root()->getConfig()["sources"].size() > 0) {
		devices = ftl::createArray<Source>(screen->root(), "sources", screen->control()->getNet());
		auto *gen = createSourceGenerator(screen->root(), devices);
		gen->onFrameSet([this, ftl_count](ftl::rgbd::FrameSet &fs) {
			fs.id = ftl_count;  // Set a frameset id to something unique.
			return _processFrameset(fs, false);
		});
	}

	stream_->begin();
}

bool SourceWindow::_processFrameset(ftl::rgbd::FrameSet &fs, bool fromstream) {
	// Request the channels required by current camera configuration
	if (fromstream) {
		auto cs = _aggregateChannels(fs.id);

		auto avail = static_cast<const ftl::stream::Stream*>(interceptor_)->available(fs.id);
		if (cs.has(Channel::Depth) && !avail.has(Channel::Depth) && avail.has(Channel::Right)) {
			cs -= Channel::Depth;
			cs += Channel::Right;
		}
		interceptor_->select(fs.id, cs);
	}

	// Make sure there are enough framesets allocated
	_checkFrameSets(fs.id);

	if (!paused_) {
		// Enforce interpolated colour and GPU upload
		for (int i=0; i<fs.frames.size(); ++i) {
			fs.frames[i].createTexture<uchar4>(Channel::Colour, true);

			// TODO: Do all channels. This is a fix for screen capture sources.
			if (!fs.frames[i].isGPU(Channel::Colour)) fs.frames[i].upload(Channels<0>(Channel::Colour), pre_pipelines_[fs.id]->getStream());
		}

		{
			FTL_Profile("Prepipe",0.020);
			pre_pipelines_[fs.id]->apply(fs, fs, 0);
		}

		fs.swapTo(*framesets_[fs.id]);
	}

	const auto *cstream = interceptor_;
	_createDefaultCameras(*framesets_[fs.id], true);  // cstream->available(fs.id).has(Channel::Depth)

	//LOG(INFO) << "Channels = " << (unsigned int)cstream->available(fs.id);

	int i=0;
	for (auto cam : cameras_) {
		// Only update the camera periodically unless the active camera
		if (screen_->activeCamera() == cam.second.camera ||
			(screen_->activeCamera() == nullptr && cycle_ % cameras_.size() == i++))  cam.second.camera->update(framesets_);

		ftl::codecs::Channels<0> channels;
		if (fromstream) channels = cstream->available(fs.id);
		if ((*framesets_[fs.id]).frames.size() > 0) channels += (*framesets_[fs.id]).frames[0].getChannels();
		cam.second.camera->update(fs.id, channels);
	}
	++cycle_;

	return true;
}

void SourceWindow::_checkFrameSets(int id) {
	while (framesets_.size() <= id) {
		auto *p = ftl::config::create<ftl::operators::Graph>(screen_->root(), "pre_filters");
		p->append<ftl::operators::DepthChannel>("depth")->value("enabled", false);
		//p->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
		p->append<ftl::operators::DetectAndTrack>("facedetection")->value("enabled", false);
		p->append<ftl::operators::ArUco>("aruco")->value("enabled", false);
		//p->append<ftl::operators::HFSmoother>("hfnoise");
		p->append<ftl::operators::CrossSupport>("cross");
		p->append<ftl::operators::PixelWeights>("weights");
		p->append<ftl::operators::CullWeight>("remove_weights")->value("enabled", false);
		p->append<ftl::operators::DegradeWeight>("degrade");
		p->append<ftl::operators::VisCrossSupport>("viscross")->set("enabled", false);
		p->append<ftl::operators::CullDiscontinuity>("remove_discontinuity");
		p->append<ftl::operators::MultiViewMLS>("mvmls")->value("enabled", false);

		pre_pipelines_.push_back(p);
		framesets_.push_back(new ftl::rgbd::FrameSet);
	}
}

void SourceWindow::recordVideo(const std::string &filename) {
	if (!recorder_->active()) {
		recorder_->set("filename", filename);
		recorder_->begin();
		LOG(INFO) << "Recording started: " << filename;

		// TODO: Inject pose and calibrations
		stream_->reset();
	}
}

void SourceWindow::stopRecordingVideo() {
	if (recorder_->active()) {
		recorder_->end();
		LOG(INFO) << "Recording stopped.";
	}
}

ftl::codecs::Channels<0> SourceWindow::_aggregateChannels(int id) {
	ftl::codecs::Channels<0> cs = ftl::codecs::Channels<0>(Channel::Colour);
	for (auto cam : cameras_) {
		if (cam.second.camera->usesFrameset(id)) {
			if (cam.second.camera->isVirtual()) {
				cs += Channel::Depth;
			} else {
				if (cam.second.camera->getChannel() != Channel::None) {
					cs += cam.second.camera->getChannel();
				}
			}
		}
	}

	return cs;
}

void SourceWindow::_createDefaultCameras(ftl::rgbd::FrameSet &fs, bool makevirtual) {
	for (int i=0; i<fs.frames.size(); ++i) {
		int id = (fs.id << 8) + i;
		if (cameras_.find(id) == cameras_.end()) {
			auto *cam = new ftl::gui::Camera(screen_, 1 << fs.id, i);
			cameras_[id] = {
				cam,
				nullptr
			};
		}
	}

	if (makevirtual && cameras_.find((fs.id << 8) + 255) == cameras_.end()) {
		auto *cam = new ftl::gui::Camera(screen_, 1 << fs.id, 255);
		cameras_[(fs.id << 8) + 255] = {
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
		bool isspecial = (stream->get<std::string>("uri") == screen_->root()->value("data_stream",std::string("")));
		if (isspecial) LOG(INFO) << "Adding special stream";
		stream_->add(stream, (isspecial) ? 1 : 0);

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
				//cam->draw(framesets_);
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
