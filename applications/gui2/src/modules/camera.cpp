#include "camera.hpp"
#include "statistics.hpp"

#include "../views/camera3d.hpp"
#include <ftl/rgbd/capabilities.hpp>
#include <ftl/streams/renderer.hpp>
#include <chrono>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/cudaarithm.hpp>

#include <loguru.hpp>

using ftl::gui2::Camera;
using ftl::codecs::Channel;
using ftl::rgbd::Capability;
using namespace std::literals::chrono_literals;
using ftl::data::Message;

void Camera::init() {

	colouriser_ = std::unique_ptr<ftl::render::Colouriser>(
		ftl::create<ftl::render::Colouriser>(this, "colouriser"));

	overlay_ = std::unique_ptr<ftl::overlay::Overlay>
		(ftl::create<ftl::overlay::Overlay>(this, "overlay"));
}

void Camera::update(double delta) {
	if (nframes_ < 0) { return; }
	if (nframes_ > update_fps_freq_) {
		float n = nframes_;
		float l = latency_ / n;
		nframes_ = 0;
		latency_ = 0;
		auto t =  glfwGetTime();
		float diff = t - last_;
		last_ =  t;

		auto *mod = screen->getModule<ftl::gui2::Statistics>();
		mod->getJSON(StatisticsPanel::PERFORMANCE_INFO)["FPS"] = n/diff;
		mod->getJSON(StatisticsPanel::PERFORMANCE_INFO)["Latency"] = std::to_string(int(l))+std::string("ms");
		if (live_) mod->getJSON(StatisticsPanel::MEDIA_STATUS)["LIVE"] = nlohmann::json{{"icon", ENTYPO_ICON_VIDEO_CAMERA},{"value", true},{"colour","#0000FF"},{"size",28}};

		auto ptr = std::atomic_load(&latest_);
		if (ptr) {
			const auto &frame = ptr->frames[frame_idx];
			if (frame.has(Channel::MetaData)) {
				const auto &meta = frame.metadata();
				if (meta.size() > 0) {
					auto &jmeta = mod->getJSON(StatisticsPanel::MEDIA_META);

					//if (meta.count("name")) {
					//	jmeta["name"] = nlohmann::json{{"nokey", true},{"value",meta.find("name")->second},{"size",20}};
					//}
					if (meta.count("device")) {
						jmeta["Device"] = nlohmann::json{{"nokey", true},{"value",meta.find("device")->second}};
					}
					if (meta.count("serial")) {
						jmeta["Serial"] = nlohmann::json{{"value",meta.find("serial")->second}};
					}

					/*for (const auto &m : meta) {
						jmeta[m.first] = m.second;
					}*/
				}
			}

			const auto &rgbdf = frame.cast<ftl::rgbd::Frame>();

			if (frame.has(Channel::Calibration)) {
				const auto &cam = rgbdf.getLeft();
				auto &jcam = mod->getJSON(StatisticsPanel::CAMERA_DETAILS);
				jcam["Resolution"] = std::to_string(cam.width) + std::string("x") + std::to_string(cam.height);
				jcam["Focal"] = cam.fx;
				jcam["Principle"] = std::to_string(int(cam.cx)) + std::string(",") + std::to_string(int(cam.cy));
			}

			if (frame.has(Channel::Capabilities)) {
				const auto &caps = rgbdf.capabilities();
				auto &jmeta = mod->getJSON(StatisticsPanel::MEDIA_META);

				if (caps.count(Capability::TOUCH)) jmeta["Touch"] = nlohmann::json{{"icon", ENTYPO_ICON_MOUSE_POINTER},{"value", true}};
				else jmeta.erase("Touch");

				if (caps.count(Capability::MOVABLE)) jmeta["Movable"] = nlohmann::json{{"icon", ENTYPO_ICON_DIRECTION},{"value", true}};
				else jmeta.erase("Movable");

				if (caps.count(Capability::VR)) jmeta["VR"] = nlohmann::json{{"value", true}};
				else jmeta.erase("VR");

				if (caps.count(Capability::EQUI_RECT)) jmeta["360"] = nlohmann::json{{"icon", ENTYPO_ICON_COMPASS},{"value", true}};
				else jmeta.erase("360");
			}

			std::map<ftl::data::Message,std::string> messages;
			{
				UNIQUE_LOCK(mtx_, lk);
				std::swap(messages, messages_);
			}

			auto &jmsgs = mod->getJSON(StatisticsPanel::LOGGING);
			jmsgs.clear();
			if (messages.size() > 0) {
				for (const auto &m : messages) {
					auto &data = jmsgs.emplace_back();
					data["value"] = m.second;
					data["nokey"] = true;
					if (int(m.first) < 1024) {
						data["icon"] = ENTYPO_ICON_WARNING;
						data["colour"] = "#0000ff";
					} else if (int(m.first) < 2046) {
						data["icon"] = ENTYPO_ICON_WARNING;
						data["colour"] = "#00a6f0";
					} else {

					}
				}
			}
		}
	}
}

void Camera::_updateCapabilities(ftl::data::Frame &frame) {
	if (frame.has(Channel::Capabilities)) {
		live_ = false;
		touch_ = false;
		movable_ = false;
		vr_ = false;

		const auto &cap = frame.get<std::unordered_set<Capability>>(Channel::Capabilities);

		for (auto c : cap) {
			switch (c) {
			case Capability::LIVE		: live_ = true; break;
			case Capability::TOUCH		: touch_ = true; break;
			case Capability::MOVABLE	: movable_ = true; break;
			case Capability::VR			: vr_ = true; break;
			default: break;
			}
		}
	}
}

void Camera::initiate_(ftl::data::Frame &frame) {
	if (frame.has(Channel::Capabilities)) {
		const auto &rgbdf = frame.cast<ftl::rgbd::Frame>();
		const auto &cap = rgbdf.capabilities();
		for (auto c : cap) {
			LOG(INFO) << " -- " << ftl::rgbd::capabilityName(c);

			switch (c) {
			case Capability::LIVE		: live_ = true; break;
			case Capability::TOUCH		: touch_ = true; break;
			case Capability::MOVABLE	: movable_ = true; break;
			case Capability::VR			: vr_ = true; break;
			default: break;
			}
		}

		if (cap.count(Capability::VIRTUAL)) {
			view = new ftl::gui2::CameraView3D(screen, this);
		} else {
			view = new ftl::gui2::CameraView(screen, this);
		}
	} else {
		view = new ftl::gui2::CameraView(screen, this);
	}

	if (frame.has(Channel::MetaData)) {
		const auto &meta = frame.metadata();
		LOG(INFO) << "Camera Frame Meta Data:";
		for (auto m : meta) {
			LOG(INFO) << " -- " << m.first << " = " << m.second;
		}
	}

	if (!view) return;

	view->onClose([this](){
		filter_->remove();
		filter_ = nullptr;
		nframes_ = -1;

		auto *mod = this->screen->getModule<ftl::gui2::Statistics>();

		mod->getJSON(StatisticsPanel::PERFORMANCE_INFO).clear();
		mod->getJSON(StatisticsPanel::MEDIA_STATUS).clear();
		mod->getJSON(StatisticsPanel::MEDIA_META).clear();
		mod->getJSON(StatisticsPanel::CAMERA_DETAILS).clear();
	});

	setChannel(channel_);

	screen->setView(view);
	view->refresh();
}

float Camera::volume() {
	return io->speaker()->volume();
}

void Camera::setVolume(float v) {
	return io->speaker()->setVolume(v);
}

std::unordered_set<Channel> Camera::availableChannels() {
	if (std::atomic_load(&latest_)) {
		return latest_->frames[frame_idx].available();
	}
	return {};
}

std::unordered_set<Channel> Camera::allAvailableChannels() {
	if (std::atomic_load(&latest_)) {
		auto set = latest_->frames[frame_idx].available();
		for (auto i : latest_->frames[frame_idx].allChannels()) {
			set.emplace(i);
		}
		return set;
	}
	return {};
}

void Camera::activate(ftl::data::FrameID id) {
	frame_idx = id.source();
	frame_id_ = id;
	last_ = glfwGetTime();
	nframes_ = 0;
	// Clear the members to defaults
	has_seen_frame_ = false;
	point_.id = -1;
	live_ = false;
	touch_ = false;
	movable_ = false;
	vr_ = false;

	//std::mutex m;
	//std::condition_variable cv;

	filter_ = io->feed()->filter(std::unordered_set<unsigned int>{id.frameset()}, {Channel::Left});
	filter_->on(
		[this, speaker = io->speaker()](ftl::data::FrameSetPtr fs){
			if (paused_) return true;

			std::atomic_store(&current_fs_, fs);
			std::atomic_store(&latest_, fs);

			// Deal with audio
			if (fs->frames[frame_idx].hasOwn(Channel::AudioStereo)) {
				speaker->queue(fs->timestamp(), fs->frames[frame_idx]);
			}

			// Need to notify GUI thread when first data comes
			if (!has_seen_frame_) {
				//std::unique_lock<std::mutex> lk(m);
				has_seen_frame_ = true;
				//cv.notify_one();
			}

			// Extract and record any frame messages
			auto &frame = fs->frames[frame_idx];
			if (frame.hasMessages()) {
				const auto &msgs = frame.messages();
				//auto &jmsgs = mod->getJSON(StatisticsPanel::LOGGING);

				UNIQUE_LOCK(mtx_, lk);
				messages_.insert(msgs.begin(), msgs.end());
			}

			// Some capabilities can change over time
			if (frame.changed(Channel::Capabilities)) {
				_updateCapabilities(frame);
			}

			if (!view) return true;

			if (live_ && touch_) {
				if (point_.id >= 0) {
					auto response = fs->frames[frame_idx].response();
					auto &data = response.create<std::vector<ftl::codecs::Touch>>(Channel::Touch);
					data.resize(1);
					UNIQUE_LOCK(mtx_, lk);
					data[0] = point_;
					//point_.strength = 0;
				}
			}

			screen->redraw();
			nframes_++;
			latency_ += ftl::timer::get_time() - fs->localTimestamp;
			return true;
		}
	);

	auto sets = filter_->getLatestFrameSets();
	if (sets.size() > 0) {
		std::atomic_store(&current_fs_, sets.front());
		std::atomic_store(&latest_, sets.front());
		initiate_(sets.front()->frames[frame_idx]);
	} else {
		throw FTL_Error("Cannot activate camera, no data");
	}

	// For first data, extract useful things and create view
	// Must be done in GUI thread, hence use of cv.
	//std::unique_lock<std::mutex> lk(m);
	//cv.wait_for(lk, 1s, [this](){ return has_seen_frame_; });
	//initiate_(std::atomic_load(&current_fs_)->frames[frame_idx]);
}

void Camera::setChannel(Channel c) {
	channel_ = c;
	filter_->select({Channel::Colour, c});
}

std::string Camera::getActiveSourceURI() {
	auto ptr = std::atomic_load(&latest_);
	if (ptr) {
		auto &frame = ptr->frames[frame_idx];
		if (frame.has(ftl::codecs::Channel::MetaData)) {
			const auto &meta = frame.metadata();
			auto i = meta.find("id");
			if (i != meta.end()) {
				return i->second;
			}
		}
	}

	return "";
}

ftl::audio::StereoMixerF<100> *Camera::mixer() {
	if (mixer_) return mixer_;
	if (movable_) {
		auto *rend = io->feed()->getRenderer(frame_id_);
		if (rend) {
			mixer_ = &(rend->mixer());
			return mixer_;
		}
	}
	return nullptr;
}

bool Camera::isRecording() {
	return io->feed()->isRecording();
}

void Camera::stopRecording() {
	io->feed()->stopRecording();
	filter_->select({channel_});
}

void Camera::startRecording(const std::string &filename, const std::unordered_set<ftl::codecs::Channel> &channels) {
	filter_->select(channels);
	io->feed()->startRecording(filter_, filename);
}

void Camera::startStreaming(const std::unordered_set<ftl::codecs::Channel> &channels) {
	filter_->select(channels);
	io->feed()->startStreaming(filter_);
}

void Camera::snapshot(const std::string &filename) {
	auto ptr = std::atomic_load(&latest_);
	if (ptr) {
		auto &frame = ptr->frames[frame_idx];
		if (frame.hasChannel(channel_)) {
			const auto &snap = frame.get<cv::Mat>(channel_);
			cv::Mat output;
			cv::cvtColor(snap, output, cv::COLOR_BGRA2BGR);
			cv::imwrite(filename, output);
		}
	}
}

ftl::cuda::TextureObject<uchar4>& Camera::getFrame() {
	return getFrame(channel_);
}

ftl::cuda::TextureObject<uchar4>& Camera::getFrame(ftl::codecs::Channel channel) {
	if (std::atomic_load(&current_fs_)) {
		auto& frame = current_fs_->frames[frame_idx].cast<ftl::rgbd::Frame>();

		current_frame_colour_ = frame.getTexture<uchar4>(Channel::Left);

		if (frame.hasChannel(channel)) {
			current_frame_ = colouriser_->colourise(frame, channel, 0);
		} else {
			throw FTL_Error("Channel missing for frame " << frame.timestamp() << ": '" << ftl::codecs::name(channel) << "'");
		}
		std::atomic_store(&current_fs_, {});
	}
	if (channel == Channel::Left) { return current_frame_colour_; }
	else { return current_frame_; }
}

bool Camera::getFrame(ftl::cuda::TextureObject<uchar4>& frame, ftl::codecs::Channel channel) {
	if (std::atomic_load(&current_fs_).get() != nullptr) {
		frame = getFrame();
		return true;
	}
	return false;
}

bool Camera::getFrame(ftl::cuda::TextureObject<uchar4>& frame) {
	return getFrame(frame, channel_);
}

bool Camera::hasFrame() {
	auto ptr = std::atomic_load(&current_fs_);
	if (ptr && ptr->frames.size() > (unsigned int)(frame_idx)) {
		return ptr->frames[frame_idx].hasChannel(channel_);
	}
	return false;
}

void Camera::drawOverlay(NVGcontext *ctx) {
	auto ptr = std::atomic_load(&latest_);
	// TODO: Need all the source framesets here or all data dumped in by renderer
	overlay_->draw(ctx, *ptr, ptr->frames[frame_idx].cast<ftl::rgbd::Frame>(), view->size().cast<float>());
}

void Camera::sendPose(const Eigen::Matrix4d &pose) {
	if (live_ && movable_ && !vr_) {
		if (auto ptr = std::atomic_load(&latest_)) {
			auto response = ptr->frames[frame_idx].response();
			auto &rgbdresponse = response.cast<ftl::rgbd::Frame>();
			rgbdresponse.setPose() = pose;
		}
	}
}

void Camera::touch(int id, ftl::codecs::TouchType t, int x, int y, float d, int strength) {
	if (value("enable_touch", false)) {
		UNIQUE_LOCK(mtx_, lk);
		point_.id = id;
		point_.type = t;
		point_.x = x;
		point_.y = y;
		point_.d = d;
		point_.strength = strength; //std::max((unsigned char)strength, point_.strength);
	}

	// TODO: Check for touch capability first
	/*if (auto ptr = std::atomic_load(&latest_)) {
		auto response = ptr->frames[frame_idx].response();
		auto &data = response.create<std::vector<ftl::codecs::Touch>>(Channel::Touch);
		data.resize(0);
		auto &pt = data.emplace_back();
		pt.id = id;
		pt.type = t;
		pt.x = x;
		pt.y = y;
		pt.d = d;
		pt.strength = strength;


	}*/
}

float Camera::depthAt(int x, int y) {
	if (value("show_depth", true)) {
		auto ptr = std::atomic_load(&latest_);

		if (ptr) {
			const auto &frame = ptr->frames[frame_idx].cast<ftl::rgbd::Frame>();

			if (frame.hasChannel(Channel::Depth)) {
				const auto &depth = frame.get<cv::Mat>(Channel::Depth);
				if (x >= 0 && y >= 0 && x < depth.cols && y < depth.rows) {
					return depth.at<float>(y, x);
				}
			}
		}
	}
	return 0.0f;
}
