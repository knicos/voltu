#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/codecs/reader.hpp>
#include <ftl/codecs/decoder.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/rgbd/camera.hpp>

#include <fstream>

#include <Eigen/Eigen>

using ftl::codecs::codec_t;

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

    bool res = r.read(90000000000000, [&current_stream,&current_channel,&r](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
        if (spkt.channel != current_channel) return;
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

            LOG(INFO) << "Reading packet: (" << (int)spkt.streamID << "," << (int)spkt.channel << ") " << (int)pkt.codec << ", " << (int)pkt.definition;

            cv::Mat frame(cv::Size(ftl::codecs::getWidth(pkt.definition),ftl::codecs::getHeight(pkt.definition)), (spkt.channel == 1) ? CV_32F : CV_8UC3);
            createDecoder(pkt);

            try {
                decoder->decode(pkt, frame);
            } catch (std::exception &e) {
                LOG(INFO) << "Decoder exception: " << e.what();
            }

            if (!frame.empty()) {
                if (spkt.channel == 1) {
                    visualizeDepthMap(frame, frame, 8.0f);
                }
                double time = (double)(spkt.timestamp - r.getStartTime()) / 1000.0;
                cv::putText(frame, std::string("Time: ") + std::to_string(time) + std::string("s"), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
                cv::imshow("Player", frame);
            }
            int key = cv::waitKey(20);
            if (key >= 48 && key <= 57) {
                current_stream = key - 48;
            } else if (key == 'd') {
                current_channel = (current_channel == 0) ? 1 : 0;
            } else if (key == 27) {
                r.end();
            }
        }
    });

    if (!res) LOG(ERROR) << "No frames left";

    r.end();

	ftl::running = false;
	return 0;
}
