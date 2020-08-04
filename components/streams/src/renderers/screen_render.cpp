#include <ftl/streams/renderer.hpp>
#include <ftl/rgbd/frame.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/rgbd/capabilities.hpp>
#include <ftl/operators/antialiasing.hpp>
#include <ftl/operators/gt_analysis.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include <ftl/codecs/shapes.hpp>

#include <loguru.hpp>

#include "screen_render.hpp"
#include "collisions.hpp"

using ftl::render::Source;
using ftl::render::ScreenRender;
using ftl::codecs::Channel;
using ftl::rgbd::Capability;
using ftl::codecs::Shape3DType;


ScreenRender::ScreenRender(ftl::render::Source *host, ftl::stream::Feed *feed)
: ftl::render::BaseSourceImpl(host), feed_(feed), my_id_(0), post_pipe_(nullptr) {
	/*host->restore("device:render", {
		"renderer",
		"source",
		"intrinsics",
		"name"
	});*/

	renderer_ = std::unique_ptr<ftl::render::CUDARender>(
		ftl::create<ftl::render::CUDARender>(host_, "renderer")
	);

	intrinsics_ = ftl::create<ftl::Configurable>(host_, "intrinsics");

	intrinsics_->onAny({"focal","width","height"}, [this]() {
		calibration_uptodate_.clear();
	});

	renderer_->value("projection", 0);
	renderer_->onAny({"projection"}, [this]() {
		calibration_uptodate_.clear();
	});

	filter_ = nullptr;
	std::string source = host_->value("source", std::string(""));

	if (source.size() > 0) {
		filter_ = feed_->filter({source},{Channel::Colour, Channel::Depth});
	} else {
		filter_ = feed_->filter({Channel::Colour, Channel::Depth});
	}

	host_->on("source", [this]() {
		std::string source = host_->value("source", std::string(""));

		if (source.size() > 0) {
			if (filter_) filter_->remove();
			filter_ = feed_->filter({source},{Channel::Colour, Channel::Depth});
		} else {
			if (filter_) filter_->remove();
			filter_ = feed_->filter({Channel::Colour, Channel::Depth});
		}
	});
}

ScreenRender::~ScreenRender() {
	if (filter_) filter_->remove();
	delete intrinsics_;
	if (post_pipe_) delete post_pipe_;
}

bool ScreenRender::isReady() {
	return true;
}

bool ScreenRender::capture(int64_t ts) {
	return true;
}

bool ScreenRender::retrieve(ftl::data::Frame &frame_out) {
	//auto input = std::atomic_load(&input_);

	my_id_ = frame_out.frameset();
	auto sets = filter_->getLatestFrameSets();
	bool data_only = host_->value("data_only", false);

	if (sets.size() > 0) {
		ftl::rgbd::Frame &rgbdframe = frame_out.cast<ftl::rgbd::Frame>();

		if (!frame_out.has(Channel::Calibration) || calibration_uptodate_.test_and_set()) {
			rgbdframe.setLeft() = ftl::rgbd::Camera::from(intrinsics_);

			auto &cap = frame_out.create<std::unordered_set<Capability>>(Channel::Capabilities);
			cap.clear();
			cap.emplace(Capability::VIDEO);
			cap.emplace(Capability::MOVABLE);
			cap.emplace(Capability::ADJUSTABLE);
			cap.emplace(Capability::VIRTUAL);
			cap.emplace(Capability::LIVE);
			if (renderer_->value("projection", 0) == int(ftl::rgbd::Projection::EQUIRECTANGULAR)) cap.emplace(Capability::EQUI_RECT);

			auto &meta = frame_out.create<std::map<std::string,std::string>>(Channel::MetaData);
			meta["name"] = host_->value("name", host_->getID());
			meta["id"] = host_->getID();
			meta["uri"] = std::string("device:render");
			meta["device"] = std::string("CUDA Render");
		}
		if (!frame_out.has(Channel::Pose)) {
			rgbdframe.setPose() = Eigen::Matrix4d::Identity();
		}

		int width = rgbdframe.getLeft().width;
		int height = rgbdframe.getLeft().height;
		
		// FIXME: Create opengl buffers here and map to cuda?
		auto &colour = rgbdframe.create<cv::cuda::GpuMat>(Channel::Colour);
		colour.create(height, width, CV_8UC4);
		rgbdframe.create<cv::cuda::GpuMat>(Channel::Depth).create(height, width, CV_32F);
		rgbdframe.createTexture<float>(Channel::Depth);

		if (data_only) {
			colour.setTo(cv::Scalar(0,0,0,0));
		}

		auto shapes = rgbdframe.create<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);

		try {
			if (!data_only) renderer_->begin(rgbdframe, ftl::codecs::Channel::Left);

			// Submit jobs to GPU first
			for (auto &s : sets) {
				if (s->frameset() == my_id_) continue;  // Skip self

				Eigen::Matrix4d pose;
				pose.setIdentity();
				if (s->hasChannel(Channel::Pose)) pose = s->cast<ftl::rgbd::Frame>().getPose();

				if (!data_only) renderer_->submit(
					s.get(),
					ftl::codecs::Channels<0>(ftl::codecs::Channel::Colour),
					pose);
			}

			if (!data_only) renderer_->render();

			// Blend another channel
			int blend_channel = host_->value("blend_channel",0);
			if (blend_channel > 0) {
				if (!data_only) renderer_->blend(static_cast<Channel>(blend_channel));
			}

			// Now do CPU-based render jobs
			for (auto &s : sets) {
				if (s->frameset() == my_id_) continue;  // Skip self

				// Inject and copy data items and mix audio
				for (size_t i=0; i<s->frames.size(); ++i) {
					auto &f = s->frames[i];

					// If audio is present, mix with the other frames
					if (f.hasChannel(Channel::AudioStereo)) {
						// Map a mixer track to this frame
						auto &mixmap = mixmap_[f.id().id];
						if (mixmap.track == -1) {
							mixmap.track = mixer_.add(f.name());
						}

						// Do mix but must not mix same frame multiple times
						if (mixmap.last_timestamp != f.timestamp()) {
							const auto &audio = f.get<std::list<ftl::audio::Audio>>(Channel::AudioStereo).front();
							mixer_.write(mixmap.track, audio.data());
							mixmap.last_timestamp = f.timestamp();
						}
					}

					// Add pose as a camera shape
					auto &shape = shapes.list.emplace_back();
					shape.id = f.id().id;
					shape.type = Shape3DType::CAMERA;
					shape.size = Eigen::Vector3f(0.2f,0.2f,0.2f);
					shape.pose = f.cast<ftl::rgbd::Frame>().getPose().cast<float>();
					shape.label = f.name();

					// Copy all original shapes
					if (f.hasChannel(Channel::Shapes3D)) {
						const auto &fshapes = f.get<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);
						shapes.list.insert(shapes.list.end(), fshapes.begin(), fshapes.end());
					}
				}
			}

			mixer_.mix();

			// Write mixed audio to frame.
			if (mixer_.frames() > 0) {
				auto &list = frame_out.create<std::list<ftl::audio::Audio>>(Channel::AudioStereo).list;
				list.clear();

				int fcount = mixer_.frames();
				mixer_.read(list.emplace_front().data(), fcount);
			}

			// This waits for GPU also
			if (!data_only) renderer_->end();
		} catch (const ftl::exception &e) {
			LOG(ERROR) << "Render exception: " << e.what();
			renderer_->cancel();
			frame_out.message(ftl::data::Message::Error_RENDER, e.what());
		}

		if (!data_only) {
			if (!post_pipe_) {
				post_pipe_ = ftl::config::create<ftl::operators::Graph>(host(), "post_filters");
				post_pipe_->append<ftl::operators::FXAA>("fxaa");
				post_pipe_->append<ftl::operators::GTAnalysis>("gtanalyse");
			}

			post_pipe_->apply(rgbdframe, rgbdframe, 0);

			if (host_->value("enable_touch", false)) {
				ftl::render::collision2touch(rgbdframe, renderer_->getCollisions(), sets, my_id_, host_->value("touch_min", 0.01f), host_->value("touch_max", 0.05f));
			}
		}

		return true;
	} else {
		//LOG(INFO) << "Render fail";
		return true;
	}
}
