#include "developer.hpp"
#include <loguru.hpp>

#include "../../views/dev/disparityview.hpp"
#include <ftl/codecs/channels.hpp>

using ftl::gui2::DisparityDev;
using ftl::codecs::Channel;
using ftl::rgbd::Capability;

void DisparityDev::init() {
	colouriser_ = std::unique_ptr<ftl::render::Colouriser>(
		ftl::create<ftl::render::Colouriser>(this, "colouriser"));
}

void DisparityDev::_updateCapabilities(ftl::data::Frame &frame) {
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

void DisparityDev::initiate_(ftl::data::Frame &frame) {
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
			//view = new ftl::gui2::CameraView3D(screen, this);
		} else {
			//view = new ftl::gui2::CameraView(screen, this);
		}
	} else {
		///view = new ftl::gui2::CameraView(screen, this);
	}

	has_seen_frame_ = true;
	view = new ftl::gui2::DisparityView(screen, this);

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

		/*auto *mod = this->screen->getModule<ftl::gui2::Statistics>();

		mod->getJSON(StatisticsPanel::PERFORMANCE_INFO).clear();
		mod->getJSON(StatisticsPanel::MEDIA_STATUS).clear();
		mod->getJSON(StatisticsPanel::MEDIA_META).clear();
		mod->getJSON(StatisticsPanel::CAMERA_DETAILS).clear();*/
	});

	screen->setView(view);
	view->refresh();
}

void DisparityDev::activate(ftl::data::FrameID id) {
	LOG(INFO) << "DISP DEV ACTIVATE";
	frame_idx = id.source();
	frame_id_ = id;
	last_ = glfwGetTime();
	nframes_ = 0;
	// Clear the members to defaults
	has_seen_frame_ = false;
	live_ = false;
	touch_ = false;
	movable_ = false;
	vr_ = false;

	filter_ = io->feed()->filter(std::unordered_set<unsigned int>{id.frameset()}, {Channel::Left, Channel::Right});
	filter_->on(
		[this, feed = io->feed(), speaker = io->speaker()](ftl::data::FrameSetPtr fs){
			std::atomic_store(&current_fs_, fs);
			std::atomic_store(&latest_, fs);

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
		throw FTL_Error("Cannot activate disparity devtools, no data");
	}
}

DisparityDev::~DisparityDev() {

}

ftl::cuda::TextureObject<uchar4>& DisparityDev::getFrame() {
	return getFrame(Channel::Right);
}

ftl::cuda::TextureObject<uchar4>& DisparityDev::getFrame(ftl::codecs::Channel channel) {
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

bool DisparityDev::getFrame(ftl::cuda::TextureObject<uchar4>& frame, ftl::codecs::Channel channel) {
	if (std::atomic_load(&current_fs_).get() != nullptr) {
		frame = getFrame();
		return true;
	}
	return false;
}

bool DisparityDev::getFrame(ftl::cuda::TextureObject<uchar4>& frame) {
	return getFrame(frame, Channel::Right);
}

bool DisparityDev::hasFrame() {
	auto ptr = std::atomic_load(&current_fs_);
	if (ptr && ptr->frames.size() > (unsigned int)(frame_idx)) {
		return ptr->frames[frame_idx].hasChannel(Channel::Left);
	}
	return false;
}

void DisparityDev::generate() {
	auto ptr = std::atomic_load(&current_fs_);
	if (ptr && ptr->frames.size() > (unsigned int)(frame_idx)) {
		if (ptr->frames[frame_idx].hasChannel(Channel::Left)) {
			col_feat_left_.generate(ptr->frames[frame_idx].get<cv::cuda::GpuMat>(Channel::Left), nullptr);
		}
		if (ptr->frames[frame_idx].hasChannel(Channel::Right)) {
			col_feat_right_.generate(ptr->frames[frame_idx].get<cv::cuda::GpuMat>(Channel::Right), nullptr);
		}
	}
}

const cv::cuda::GpuMat& DisparityDev::getFeatureImageLeft(ftl::disparity::ColourFeatures::Feature f) {
	col_feat_left_.visualise(f, 0, left_, nullptr);
	return left_;
}

const cv::cuda::GpuMat& DisparityDev::getFeatureImageRight(ftl::disparity::ColourFeatures::Feature f) {
	col_feat_right_.visualise(f, 0, right_, nullptr);
	return right_;
}
