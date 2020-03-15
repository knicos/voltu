#include <ftl/ftl.h>
#include <ftl/uri.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/streams/sender.hpp>
#include <ftl/streams/filestream.hpp>
#include <ftl/streams/netstream.hpp>

#include <opencv2/imgproc.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

static ftlError_t last_error = FTLERROR_OK;
static ftl::Configurable *root = nullptr;

struct FTLStream {
	bool readonly;
	ftl::stream::Sender *sender;
	ftl::stream::Stream *stream;
	ftlError_t last_error;
	int64_t interval;
	bool has_fresh_data;

	std::vector<ftl::rgbd::FrameState> video_states;
	ftl::rgbd::FrameSet video_fs;
};

ftlError_t ftlGetLastStreamError(ftlStream_t stream) {
	return (stream == nullptr) ? last_error : stream->last_error;
}

static void createFileWriteStream(FTLStream *s, const ftl::URI &uri) {
	if (!root) {
		int argc = 1;
		const char *argv[] = {"SDK",0};
		root = ftl::configure(argc, const_cast<char**>(argv), "sdk_default");
	}

	auto *fs = ftl::create<ftl::stream::File>(root, "ftlfile");
	fs->set("filename", uri.getPath());
	fs->setMode(ftl::stream::File::Mode::Write);
	s->stream = fs;
};

ftlStream_t ftlCreateWriteStream(const char *uri) {
	std::string uristr(uri);
	ftl::URI u(uristr);

	if (!u.isValid()) {
		last_error = FTLERROR_STREAM_BAD_URI;
		return nullptr;
	}

	FTLStream *s = new FTLStream;
	s->last_error = FTLERROR_OK;
	s->stream = nullptr;
	s->sender = nullptr;
	s->video_fs.id = 0;
	s->video_fs.count = 0;
	s->video_fs.mask = 0;
	s->interval = 40;
	s->video_fs.frames.reserve(32);
	s->video_states.resize(32);
	s->has_fresh_data = false;

	switch (u.getScheme()) {
		case ftl::URI::SCHEME_FILE	: createFileWriteStream(s, u); break;
		default						: last_error = FTLERROR_STREAM_BAD_URI;
									  return nullptr;
	}

	if (s->last_error == FTLERROR_OK) {
		s->sender = ftl::create<ftl::stream::Sender>(root, "sender");
		//s->sender->set("codec", 0);
		s->sender->setStream(s->stream);
		if (!s->stream->begin()) {
			last_error = FTLERROR_STREAM_FILE_CREATE_FAILED;
			return nullptr;
		}
	}
	last_error = FTLERROR_OK;

	s->video_fs.timestamp = ftl::timer::get_time();

	return s;
}

ftlError_t ftlImageWrite(
	ftlStream_t stream,
	int32_t sourceId,
	ftlChannel_t channel,
	ftlImageFormat_t type,
	uint32_t pitch,
	const void *data)
{
	if (!stream || !stream->stream)
		return FTLERROR_STREAM_INVALID_STREAM;
	if (sourceId < 0 || sourceId >= 32)
		return FTLERROR_STREAM_INVALID_PARAMETER;
	if (static_cast<int>(channel) < 0 || static_cast<int>(channel) > 32)
		return FTLERROR_STREAM_BAD_CHANNEL;
	if (!stream->video_fs.hasFrame(sourceId))
		return FTLERROR_STREAM_NO_INTRINSICS;
	if (!data) return FTLERROR_STREAM_NO_DATA;
	if (stream->video_fs.hasChannel(static_cast<ftl::codecs::Channel>(channel)))
		return FTLERROR_STREAM_DUPLICATE;

	stream->sender->set("encoder_device", 2);  // Software encoder

	try {
		auto &frame = stream->video_fs.frames[sourceId];
		auto &img = frame.create<cv::cuda::GpuMat>(static_cast<ftl::codecs::Channel>(channel));
		auto &intrin = frame.getLeft();

		if (intrin.width == 0) {
			return FTLERROR_STREAM_NO_INTRINSICS;
		}

		cv::Mat tmp,tmp2;

		switch (type) {
		case FTLIMAGE_FLOAT		:	tmp2 = cv::Mat(intrin.height, intrin.width, CV_32F, const_cast<void*>(data), pitch); break;
		case FTLIMAGE_BGRA		:	tmp2 = cv::Mat(intrin.height, intrin.width, CV_8UC4, const_cast<void*>(data), pitch); break;
		case FTLIMAGE_RGBA		:	tmp = cv::Mat(intrin.height, intrin.width, CV_8UC4, const_cast<void*>(data), pitch);
									cv::cvtColor(tmp, tmp2, cv::COLOR_RGBA2BGRA);
									break;
		case FTLIMAGE_BGR		:	tmp = cv::Mat(intrin.height, intrin.width, CV_8UC3, const_cast<void*>(data), pitch);
									cv::cvtColor(tmp, tmp2, cv::COLOR_BGR2BGRA);
									break;
		case FTLIMAGE_RGB		:	tmp = cv::Mat(intrin.height, intrin.width, CV_8UC3, const_cast<void*>(data), pitch);
									cv::cvtColor(tmp, tmp2, cv::COLOR_RGB2BGRA);
									break;
		default					: return FTLERROR_STREAM_BAD_IMAGE_TYPE;
		}

		if (tmp2.empty()) return FTLERROR_STREAM_NO_DATA;
		img.upload(tmp2);

		ftl::codecs::Channels<0> channels;
		if (stream->stream->size() > static_cast<unsigned int>(stream->video_fs.id)) channels = stream->stream->selected(stream->video_fs.id);
		channels += static_cast<ftl::codecs::Channel>(channel);
		stream->stream->select(stream->video_fs.id, channels, true);

	} catch (const std::exception &e) {
		return FTLERROR_UNKNOWN;
	}

	stream->has_fresh_data = true;
	return FTLERROR_OK;
}

ftlError_t ftlIntrinsicsWriteLeft(ftlStream_t stream, int32_t sourceId, int32_t width, int32_t height, float f, float cx, float cy, float baseline, float minDepth, float maxDepth) {
	if (!stream || !stream->stream)
		return FTLERROR_STREAM_INVALID_STREAM;
	if (sourceId < 0 || sourceId >= 32)
		return FTLERROR_STREAM_INVALID_PARAMETER;

	while (stream->video_fs.frames.size() <= static_cast<unsigned int>(sourceId)) {
		stream->video_fs.frames.emplace_back();
	}

	if (stream->video_fs.hasFrame(sourceId)) {
		return FTLERROR_STREAM_DUPLICATE;
	}

	ftl::rgbd::Camera cam;
	cam.fx = f;
	cam.fy = f;
	cam.cx = cx;
	cam.cy = cy;
	cam.width = width;
	cam.height = height;
	cam.minDepth = minDepth;
	cam.maxDepth = maxDepth;
	cam.baseline = baseline;
	cam.doffs = 0.0f;
	stream->video_fs.mask |= 1 << sourceId;
	stream->video_fs.count++;
	if (!stream->video_fs.frames[sourceId].origin()) {
		stream->video_fs.frames[sourceId].setOrigin(&stream->video_states[sourceId]);
	}
	stream->video_fs.frames[sourceId].setLeft(cam);
	stream->has_fresh_data = true;

	return FTLERROR_OK;
}

ftlError_t ftlIntrinsicsWriteRight(ftlStream_t stream, int32_t sourceId, int32_t width, int32_t height, float f, float cx, float cy, float baseline, float minDepth, float maxDepth) {
	if (!stream || !stream->stream)
		return FTLERROR_STREAM_INVALID_STREAM;
	if (sourceId < 0 || sourceId >= 32)
		return FTLERROR_STREAM_INVALID_PARAMETER;
	if (!stream->video_fs.hasFrame(sourceId))
		return FTLERROR_STREAM_NO_INTRINSICS;

	ftl::rgbd::Camera cam;
	cam.fx = f;
	cam.fy = f;
	cam.cx = cx;
	cam.cy = cy;
	cam.width = width;
	cam.height = height;
	cam.minDepth = minDepth;
	cam.maxDepth = maxDepth;
	cam.baseline = baseline;
	cam.doffs = 0.0f;
	stream->video_fs.frames[sourceId].setRight(cam);
	stream->has_fresh_data = true;

	return FTLERROR_OK;
}

ftlError_t ftlPoseWrite(ftlStream_t stream, int32_t sourceId, const float *data) {
	if (!stream) return FTLERROR_STREAM_INVALID_STREAM;
	if (!stream->stream) return FTLERROR_STREAM_INVALID_STREAM;
	if (sourceId < 0 || sourceId >= 32)
		return FTLERROR_STREAM_INVALID_PARAMETER;
	if (!stream->video_fs.hasFrame(sourceId))
		return FTLERROR_STREAM_NO_INTRINSICS;
	if (!data) return FTLERROR_STREAM_NO_DATA;

	Eigen::Matrix4f pose;
	for (int i=0; i<16; ++i) pose.data()[i] = data[i];

	auto &frame = stream->video_fs.frames[sourceId];
	frame.setPose(pose.cast<double>());

	return FTLERROR_OK;
}

ftlError_t ftlSetFrameRate(ftlStream_t stream, float fps) {
	if (!stream) return FTLERROR_STREAM_INVALID_STREAM;
	if (!stream->stream) return FTLERROR_STREAM_INVALID_STREAM;

	stream->interval = int64_t(1000.0f / fps);

	return FTLERROR_OK;
}

ftlError_t ftlNextFrame(ftlStream_t stream) {
	if (!stream) return FTLERROR_STREAM_INVALID_STREAM;
	if (!stream->stream) return FTLERROR_STREAM_INVALID_STREAM;
	if (!stream->has_fresh_data) return FTLERROR_STREAM_NO_DATA;

	try {
		cudaSetDevice(0);
		stream->sender->post(stream->video_fs);
	} catch (const std::exception &e) {
		return FTLERROR_STREAM_ENCODE_FAILED;
	}

	// Reset the frameset.
	for (size_t i=0; i<stream->video_fs.frames.size(); ++i) {
		if (!stream->video_fs.hasFrame(i)) continue;

		auto &f = stream->video_fs.frames[i];
		f.reset();
		f.setOrigin(&stream->video_states[i]);
	}

	// FIXME: These should be reset each time
	//stream->video_fs.count = 0;
	//stream->video_fs.mask = 0;
	stream->video_fs.timestamp += stream->interval;
	stream->has_fresh_data = false;
	return FTLERROR_OK;
}

ftlError_t ftlDestroyStream(ftlStream_t stream) {
	if (!stream) return FTLERROR_STREAM_INVALID_STREAM;
	if (!stream->stream) return FTLERROR_STREAM_INVALID_STREAM;

	ftl::pool.push([stream](int id) {
		//ftlError_t err = FTLERROR_OK;

		if (stream->has_fresh_data) {
			try {
				cudaSetDevice(0);
				stream->sender->post(stream->video_fs);
			} catch (const std::exception &e) {
				//err = FTLERROR_STREAM_ENCODE_FAILED;
			}
		}

		if (!stream->stream->end()) {
			//err = FTLERROR_STREAM_FILE_CREATE_FAILED;
		}
		if (stream->sender) delete stream->sender;
		delete stream->stream;
		stream->sender = nullptr;
		stream->stream = nullptr;
		delete stream;
	});
	return FTLERROR_OK;
}
