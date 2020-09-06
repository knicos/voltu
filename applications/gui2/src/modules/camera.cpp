#include "camera.hpp"
#include "statistics.hpp"

#include "../views/camera3d.hpp"
#include <ftl/rgbd/capabilities.hpp>
#include <ftl/streams/renderer.hpp>
#include <chrono>
#include <ftl/utility/matrix_conversion.hpp>
#include <ftl/calibration/structures.hpp>
#include <ftl/calibration/parameters.hpp>
#include <ftl/codecs/shapes.hpp>
#include <ftl/operators/poser.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/core/eigen.hpp>

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
				cv::Size s = rgbdf.getSize();
				auto &jcam = mod->getJSON(StatisticsPanel::CAMERA_DETAILS);
				jcam["D-Resolution"] = std::to_string(cam.width) + std::string("x") + std::to_string(cam.height);
				jcam["C-Resolution"] = std::to_string(s.width) + std::string("x") + std::to_string(s.height);
				jcam["Focal"] = cam.fx;
				jcam["Baseline"] = cam.baseline;
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

		if (live_ && cap.count(Capability::VIRTUAL)) {
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
	cursor_pos_.setZero();
	cursor_normal_.setZero();
	cursor_normal_[2] = 1.0f;

	//std::mutex m;
	//std::condition_variable cv;

	io->speaker()->reset();
	io->feed()->mixer().reset();

	filter_ = io->feed()->filter(std::unordered_set<unsigned int>{id.frameset()}, {Channel::Left});
	filter_->on(
		[this, feed = io->feed(), speaker = io->speaker()](ftl::data::FrameSetPtr fs){
			if (paused_) return true;

			std::atomic_store(&current_fs_, fs);
			std::atomic_store(&latest_, fs);

			// Deal with audio
			//if (fs->frames[frame_idx].hasOwn(Channel::AudioStereo)) {
			//	speaker->queue(fs->timestamp(), fs->frames[frame_idx]);
			//}

			if (feed->mixer().frames() > 0) {
				ftl::audio::Audio aframe;
				feed->mixer().read(aframe.data(), feed->mixer().frames());
				speaker->queue(fs->timestamp(), aframe);
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

void Camera::toggleOverlay() {
	overlay_->set("enabled", !overlay_->value<bool>("enabled", false));
}

ftl::audio::StereoMixerF<100> *Camera::mixer() {
	return &io->feed()->mixer();
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

		if (frame.hasChannel(Channel::Left)) current_frame_colour_ = frame.getTexture<uchar4>(Channel::Left);

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

const Eigen::Matrix4d &Camera::cursor() const {
	return cursor_;
}

Eigen::Matrix4d Camera::_cursor() const {
	if (cursor_normal_.norm() > 0.0f) return nanogui::lookAt(cursor_pos_, cursor_target_, cursor_normal_).cast<double>();

	Eigen::Matrix4d ident;
	ident.setIdentity();
	return ident;
}

void Camera::drawOverlay(NVGcontext *ctx, const nanogui::Vector2f &s, const nanogui::Vector2f &is, const Eigen::Vector2f &offset) {
	auto ptr = std::atomic_load(&latest_);
	// TODO: Need all the source framesets here or all data dumped in by renderer
	overlay_->draw(ctx, *ptr, ptr->frames[frame_idx].cast<ftl::rgbd::Frame>(), s, is, offset, cursor());  // , view->size().cast<float>()
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
	//if (value("show_depth", true)) {
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
	//}
	return 0.0f;
}

static float3 getWorldPoint(const cv::Mat &depth, int x, int y, const ftl::rgbd::Camera &intrins, const Eigen::Matrix4f &pose) {
	if (x >= 0 && y >= 0 && x < depth.cols && y < depth.rows) {
		float d = depth.at<float>(y, x);

		if (d > intrins.minDepth && d < intrins.maxDepth) {
			float3 cam = intrins.screenToCam(x, y, d);
			float3 world = MatrixConversion::toCUDA(pose) * cam;
			return world;
		}
	}
	return make_float3(0.0f,0.0f,0.0f);
}


Eigen::Vector3f Camera::worldAt(int x, int y) {
	auto ptr = std::atomic_load(&latest_);

	Eigen::Vector3f res;
	res.setZero();

	if (ptr) {
		const auto &frame = ptr->frames[frame_idx].cast<ftl::rgbd::Frame>();

		if (frame.hasChannel(Channel::Depth)) {
			const auto &depth = frame.get<cv::Mat>(Channel::Depth);
			const auto &intrins = frame.getLeft();
			Eigen::Matrix4f posef = frame.getPose().cast<float>();

			float3 CC = getWorldPoint(depth, x, y, intrins, posef);
			res[0] = CC.x;
			res[1] = CC.y;
			res[2] = CC.z;
		}
	}

	return res;
}

Eigen::Vector3f fitPlane(const std::vector<float3>& pts) {
	// PCA: calculate covariance matrix and its eigenvectors. Eigenvector
	// corresponding to smallest eigenvalue is the plane normal.

	Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>
		mat((float*)(pts.data()), pts.size(), 3);

	Eigen::MatrixXf centered = mat.rowwise() - mat.colwise().mean();
	Eigen::MatrixXf cov = (centered.adjoint() * centered) / double(mat.rows() - 1);
	Eigen::EigenSolver<Eigen::MatrixXf> es(cov);

	Eigen::VectorXf::Index argmin;
	es.eigenvalues().real().minCoeff(&argmin);

	Eigen::Vector3cf n(es.eigenvectors().col(argmin)); // already normalized
	return n.real();
}

void Camera::setCursor(int x, int y) {
	auto ptr = std::atomic_load(&latest_);

	cursor_pos_.setZero();

	if (ptr) {
		const auto &frame = ptr->frames[frame_idx].cast<ftl::rgbd::Frame>();

		if (frame.hasChannel(Channel::Depth)) {
			const auto &depth = frame.get<cv::Mat>(Channel::Depth);
			const auto &intrins = frame.getLeft();
			Eigen::Matrix4f posef = frame.getPose().cast<float>();

			float3 CC = getWorldPoint(depth, x, y, intrins, posef);
			cursor_pos_[0] = CC.x;
			cursor_pos_[1] = CC.y;
			cursor_pos_[2] = CC.z;

			// get points around the selected point. candidates are selected in
			// from square [-range, range] around (x, y) and points which are
			// closer than max_distance are used. TODO: check bounds (depth map
			// size)
			const int range = 24; // 49x49 pixels square
			const float max_distance = 0.075; // 15cm radius
			const int min_points = 16;
			std::vector<float3> pts;
			pts.reserve((range*2 + 1)*(range*2 + 1));
			for (int xi = -range; xi <= range; xi++) {
			for (int yi = -range; yi <= range; yi++) {
				auto p = getWorldPoint(depth, x + xi, y + yi, intrins, posef);
				if (p.x == 0 && p.y == 0 && p.z == 0.0) {
					continue;
				}
				const float3 d = p - CC;
				if (sqrtf(d.x*d.x + d.y*d.y + d.z*d.z) < max_distance) {
					pts.push_back(p);
				}
			}}
			if (pts.size() < min_points) { return; }

			cursor_normal_ = fitPlane(pts);
			// don't flip y
			if (cursor_normal_.y() < 0.0) { cursor_normal_ = -cursor_normal_; }

			// some valid value as initial value
			const float3 CP = getWorldPoint(depth, x+4, y, intrins, posef);
			setCursorTarget({CP.x, CP.y, CP.z});
		}
	}

	cursor_ = _cursor();
}

void Camera::setCursorTarget(const Eigen::Vector3f &p) {
	cursor_target_ =
		p - cursor_normal_.dot(p - cursor_pos_) * cursor_normal_;
	cursor_ = _cursor();
}

void Camera::setOriginToCursor() {
	using ftl::calibration::transform::inverse;

	// Check for valid cursor
	/*if (cursor_normal_.norm() == 0.0f) return;
	float cursor_length = (cursor_target_ - cursor_pos_).norm();
	float cursor_dist = cursor_pos_.norm();
	if (cursor_length < 0.01f || cursor_length > 5.0f) return;
	if (cursor_dist > 10.0f) return;*/

	if (movable_) {
		auto *rend = io->feed()->getRenderer(frame_id_);
		if (rend) {
			auto *filter = rend->filter();
			if (filter) {
				cv::Mat cur;
				cv::eigen2cv(cursor(), cur);
				auto fss = filter->getLatestFrameSets();
				for (auto &fs : fss) {
					if (fs->frameset() == frame_id_.frameset()) continue;

					for (auto &f : fs->frames) {
						auto response = f.response();
						auto &rgbdf = response.cast<ftl::rgbd::Frame>();
						auto &calib = rgbdf.setCalibration();

						calib = f.cast<ftl::rgbd::Frame>().getCalibration();
						// apply correction to existing one
						cv::Mat new_origin = cur*calib.origin;
						if (ftl::calibration::validate::pose(new_origin)) {
							calib.origin = new_origin;
						}
						else {
							// TODO: add error message to gui as well
							LOG(ERROR) << "Bad origin update (invalid pose)";
						}
					}
				};
			}
		}
	}

	cursor_target_ = Eigen::Vector3f(0.0f,0.0f,0.0f);
	cursor_pos_ = Eigen::Vector3f(0.0f,0.0f,0.0f);
	cursor_normal_ = Eigen::Vector3f(0.0f,0.0f,0.0f);
	cursor_ = _cursor();
}

void Camera::resetOrigin() {
	cursor_target_ = Eigen::Vector3f(0.0f,0.0f,0.0f);
	cursor_pos_ = Eigen::Vector3f(0.0f,0.0f,0.0f);
	cursor_normal_ = Eigen::Vector3f(0.0f,0.0f,0.0f);
	cursor_ = _cursor();

	if (movable_) {
		auto *rend = io->feed()->getRenderer(frame_id_);
		if (rend) {
			auto *filter = rend->filter();
			if (filter) {
				cv::Mat cur;
				cv::eigen2cv(cursor(), cur);
				auto fss = filter->getLatestFrameSets();
				for (auto &fs : fss) {
					if (fs->frameset() == frame_id_.frameset()) continue;

					for (auto &f : fs->frames) {
						auto response = f.response();
						auto &rgbdf = response.cast<ftl::rgbd::Frame>();
						auto &calib = rgbdf.setCalibration();
						calib = f.cast<ftl::rgbd::Frame>().getCalibration();
						calib.origin = cur;
					}
				};
			}
		}
	}
}

void Camera::saveCursorToPoser() {
	ftl::codecs::Shape3D shape;
	shape.type = ftl::codecs::Shape3DType::CURSOR;
	shape.id = cursor_save_id_++;
	shape.label = std::string("Cursor") + std::to_string(shape.id);
	shape.pose = cursor().inverse().cast<float>();
	shape.size = Eigen::Vector3f(0.1f,0.1f,0.1f);

	ftl::operators::Poser::add(shape, frame_id_);
}

Eigen::Matrix4d Camera::getActivePose() {
	return cursor(); //.inverse();
}

nanogui::Vector2i Camera::getActivePoseScreenCoord() {
	Eigen::Matrix4d pose = getActivePose().inverse();

	auto ptr = std::atomic_load(&latest_);
	if (ptr) {
		const auto &frame = ptr->frames[frame_idx].cast<ftl::rgbd::Frame>();
		auto campose = frame.getPose().inverse() * pose;
		float3 campos;
		campos.x = campose(0,3);
		campos.y = campose(1,3);
		campos.z = campose(2,3);

		int2 spos = frame.getLeft().camToScreen<int2>(campos);
		return nanogui::Vector2i(spos.x, spos.y);
	}

	return nanogui::Vector2i(-1,-1);
}

void Camera::transformActivePose(const Eigen::Matrix4d &pose) {
	cursor_ = pose * cursor_;
}

void Camera::setActivePose(const Eigen::Matrix4d &pose) {
	cursor_ = pose; //.inverse();
}
