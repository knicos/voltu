#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/codecs/reader.hpp>
#include <ftl/codecs/decoder.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/timer.hpp>

#include <fstream>
#include <bitset>

#include <Eigen/Eigen>

const static std::string help[] = {
	"Esc", "close",
	"0-9", "change source",
	"D",   "toggle depth",
	"S",   "save screenshot",
};

std::string time_now_string() {
	char timestamp[18];
	std::time_t t=std::time(NULL);
	std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
	return std::string(timestamp);
}

using ftl::codecs::codec_t;
using ftl::codecs::Channel;

static std::map<Channel, ftl::codecs::Decoder*> decoders;

static void createDecoder(const Channel channel, const ftl::codecs::Packet &pkt) {
	if (decoders[channel]) {
		if (!decoders[channel]->accepts(pkt)) {
			ftl::codecs::free(decoders[channel]);
		} else {
			return;
		}
	}

	decoders[channel] = ftl::codecs::allocateDecoder(pkt);
}

static void visualizeDepthMap(	const cv::Mat &depth, cv::Mat &out,
								const float max_depth)
{
	depth.convertTo(out, CV_8U, 255.0f / max_depth);
	out = 255 - out;
	//cv::Mat mask = (depth >= 39.0f); // TODO (mask for invalid pixels)
	
	applyColorMap(out, out, cv::COLORMAP_JET);
	//out.setTo(cv::Scalar(255, 255, 255), mask);
}

static std::string nameForCodec(ftl::codecs::codec_t c) {
	switch(c) {
	case codec_t::JPG 	: return "JPEG";
	case codec_t::PNG	: return "PNG";
	case codec_t::H264	: return "H264";
	case codec_t::HEVC	: return "HEVC";
	case codec_t::JSON	: return "JSON";
	case codec_t::POSE	: return "POSE";
	case codec_t::RAW	: return "RAW";
	case codec_t::CALIBRATION : return "CALIBRATION";
	case codec_t::MSGPACK : return "MSGPACK";
	default: return std::string("UNKNOWN (") + std::to_string((int)c) + std::string(")");
	}
}

int main(int argc, char **argv) {
	std::string filename(argv[1]);
	LOG(INFO) << "Playing: " << filename;

	auto root = ftl::configure(argc, argv, "player_default");

	std::ifstream f;
	f.open(filename);
	if (!f.is_open()) LOG(ERROR) << "Could not open file";

	ftl::codecs::Reader r(f);
	if (!r.begin()) LOG(ERROR) << "Bad ftl file";

	LOG(INFO) << "Playing...";

	int current_stream = 0;
	int current_channel = 0;

	int stream_mask = 0;

	static volatile bool screenshot;
	static int64_t screenshot_ts;
	static cv::Mat screenshot_color;
	static cv::Mat screenshot_depth;

	std::vector<std::bitset<128>> channel_mask;

	ftl::timer::add(ftl::timer::kTimerMain, [&current_stream,&current_channel,&r,&stream_mask,&channel_mask](int64_t ts) {
		bool res = r.read(ts, [ts, &current_stream,&current_channel,&r,&stream_mask,&channel_mask](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			if (!(stream_mask & (1 << spkt.streamID))) {
				stream_mask |= 1 << spkt.streamID;
				LOG(INFO) << " - Stream found (" << (int)spkt.streamID << ")";

				channel_mask.push_back(0);
			}

			if (!(channel_mask[spkt.streamID][(int)spkt.channel])) {
				channel_mask[spkt.streamID].set((int)spkt.channel);
				LOG(INFO) << " - Channel " << (int)spkt.channel << " found (" << (int)spkt.streamID << ")";
				LOG(INFO) << "     - Codec = " << nameForCodec(pkt.codec);
				LOG(INFO) << "     - Width = " << ftl::codecs::getWidth(pkt.definition);
				LOG(INFO) << "     - Height = " << ftl::codecs::getHeight(pkt.definition);
				LOG(INFO) << "     - Start Time = " << float(spkt.timestamp - r.getStartTime()) / 1000.0f << "(s)";
				LOG(INFO) << "     - Blocks = " << (int)pkt.block_total;
			}

			if (spkt.streamID != current_stream) { return; }

			if (pkt.codec == codec_t::POSE) {
				Eigen::Matrix4d p = Eigen::Map<Eigen::Matrix4d>((double*)pkt.data.data());
				LOG(INFO) << "Have pose: " << p;
				return;
			}

			if (pkt.codec == codec_t::CALIBRATION) {
				ftl::rgbd::Camera *camera = (ftl::rgbd::Camera*)pkt.data.data();
				LOG(INFO) << "Have calibration: " << camera->fx;
				return;
			}
			
			auto channel = static_cast<ftl::codecs::Channel>(current_channel);
			if (pkt.codec == ftl::codecs::codec_t::MSGPACK) { return; }
			//LOG(INFO) << "Reading packet: (" << (int)spkt.streamID << "," << (int)spkt.channel << ") " << (int)pkt.codec << ", " << (int)pkt.definition;

			cv::cuda::GpuMat gframe(cv::Size(ftl::codecs::getWidth(pkt.definition),ftl::codecs::getHeight(pkt.definition)), (spkt.channel == Channel::Depth) ? CV_32F : CV_8UC3);
			cv::Mat frame;
			createDecoder(spkt.channel, pkt);

			try {
				decoders[spkt.channel]->decode(pkt, gframe);
				gframe.download(frame);
			} catch (std::exception &e) {
				LOG(INFO) << "Decoder exception: " << e.what();
			}

			if (screenshot) {
				if (!screenshot_depth.empty() && !screenshot_color.empty()) {
					std::string fname = time_now_string();
					LOG(INFO) << "Screenshot saved: " << fname;
					cv::imwrite(fname + "-depth.tiff", screenshot_depth);
					cv::imwrite(fname + "-color.tiff", screenshot_color);
					screenshot = false;
					screenshot_color = cv::Mat();
					screenshot_depth = cv::Mat();
					screenshot_ts = 0;
				}
				else {
					if (screenshot_ts != ts) {
						screenshot_ts = ts;
						screenshot_color = cv::Mat();
						screenshot_depth = cv::Mat();
					}
					if (spkt.channel == Channel::Colour) { 
						frame.copyTo(screenshot_color);
					}
					else if (spkt.channel == Channel::Depth) { 
						frame.copyTo(screenshot_depth);
					}
				}
			}

			if (spkt.channel != channel) return;

			if (!frame.empty()) {
				if (spkt.channel == Channel::Depth) {
					visualizeDepthMap(frame, frame, 8.0f);
				}
				double time = (double)(spkt.timestamp - r.getStartTime()) / 1000.0;
				cv::putText(frame, std::string("Time: ") + std::to_string(time) + std::string("s"), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));

				// hotkey help text
				for (int i = 0; i < std::size(help); i += 2) {
					cv::putText(frame, help[i], cv::Point(10, 40+(i/2)*14), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(64,64,255));
					cv::putText(frame, help[i+1], cv::Point(50, 40+(i/2)*14), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(64,64,255));
				}

				cv::imshow("Player", frame);
			} else {
				frame.create(cv::Size(600,600), CV_8UC3);
				cv::imshow("Player", frame);
			}
			int key = cv::waitKey(1);
			if (key >= 48 && key <= 57) {
				int new_stream = key - 48;
				if ((0 <= new_stream) && (new_stream < channel_mask.size())) {
					current_stream = new_stream;
				}
			} else if (key == 'd') {
				current_channel = (current_channel == 0) ? 1 : 0;
			} else if (key == 'r') {
				current_channel = (current_channel == 0) ? 2 : 0;
			} else if (key == 27) {
				ftl::timer::stop(false);
			} else if (key == 115) {
				screenshot = true;
			}
		});
		if (!res) ftl::timer::stop(false);
		return res;
	});

	ftl::timer::start(true);

	r.end();

	ftl::running = false;
	return 0;
}
