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

using ftl::codecs::codec_t;
using ftl::codecs::Channel;

static ftl::codecs::Decoder *decoder;


static void createDecoder(const ftl::codecs::Packet &pkt) {
	if (decoder) {
		if (!decoder->accepts(pkt)) {
			ftl::codecs::free(decoder);
		} else {
			return;
		}
	}

	decoder = ftl::codecs::allocateDecoder(pkt);
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
	std::vector<std::bitset<128>> channel_mask;

	ftl::timer::add(ftl::timer::kTimerMain, [&current_stream,&current_channel,&r,&stream_mask,&channel_mask](int64_t ts) {
		bool res = r.read(ts, [&current_stream,&current_channel,&r,&stream_mask,&channel_mask](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			if (!(stream_mask & (1 << spkt.streamID))) {
				stream_mask |= 1 << spkt.streamID;
				LOG(INFO) << " - Stream found (" << (int)spkt.streamID << ")";

				channel_mask.push_back(0);
			}

			if (!(channel_mask[spkt.streamID][(int)spkt.channel])) {
				channel_mask[spkt.streamID].set((int)spkt.channel);
				LOG(INFO) << " - Channel " << (int)spkt.channel << " found (" << (int)spkt.streamID << ")";
			}

			if (spkt.streamID == current_stream) {

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
				
				if (spkt.channel != static_cast<ftl::codecs::Channel>(current_channel)) return;

				//LOG(INFO) << "Reading packet: (" << (int)spkt.streamID << "," << (int)spkt.channel << ") " << (int)pkt.codec << ", " << (int)pkt.definition;

				cv::cuda::GpuMat gframe(cv::Size(ftl::codecs::getWidth(pkt.definition),ftl::codecs::getHeight(pkt.definition)), (spkt.channel == Channel::Depth) ? CV_32F : CV_8UC3);
				cv::Mat frame;
				createDecoder(pkt);

				try {
					decoder->decode(pkt, gframe);
					gframe.download(frame);
				} catch (std::exception &e) {
					LOG(INFO) << "Decoder exception: " << e.what();
				}

				if (!frame.empty()) {
					if (spkt.channel == Channel::Depth) {
						visualizeDepthMap(frame, frame, 8.0f);
					}
					double time = (double)(spkt.timestamp - r.getStartTime()) / 1000.0;
					cv::putText(frame, std::string("Time: ") + std::to_string(time) + std::string("s"), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
					cv::imshow("Player", frame);
				} else {
					frame.create(cv::Size(600,600), CV_8UC3);
					cv::imshow("Player", frame);
				}
				int key = cv::waitKey(1);
				if (key >= 48 && key <= 57) {
					current_stream = key - 48;
				} else if (key == 'd') {
					current_channel = (current_channel == 0) ? 1 : 0;
				} else if (key == 'r') {
					current_channel = (current_channel == 0) ? 2 : 0;
				} else if (key == 27) {
					ftl::timer::stop(false);
				}
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
