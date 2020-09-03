#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/codecs/reader.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/codecs/hevc.hpp>
#include <ftl/codecs/h264.hpp>
#include <ftl/codecs/decoder.hpp>

#include <fstream>

#include <Eigen/Eigen>

extern "C" {
 #include <libavformat/avformat.h>
}

using ftl::codecs::codec_t;
using ftl::codecs::Channel;

static AVStream *add_video_stream(AVFormatContext *oc, const ftl::codecs::Packet &pkt, const ftl::rgbd::Camera &cam)
{
    //AVCodecContext *c;
    AVStream *st;

    st = avformat_new_stream(oc, 0);
    if (!st) {
        fprintf(stderr, "Could not alloc stream\n");
        exit(1);
    }

	AVCodecID codec_id = AV_CODEC_ID_HEVC;
	switch (pkt.codec) {
	case codec_t::HEVC_LOSSLESS:
	case codec_t::HEVC : codec_id = AV_CODEC_ID_HEVC; break;
	case codec_t::H264_LOSSLESS:
	case codec_t::H264 : codec_id = AV_CODEC_ID_H264; break;
	default: LOG(FATAL) << "Codec in FTL file must be HEVC or H264";
	}

    //c = st->codec;
    //c->codec_id = codec_id;
    //c->codec_type = AVMEDIA_TYPE_VIDEO;

	//st->time_base.den = 20;
	//st->time_base.num = 1;
	//st->id = oc->nb_streams-1;
	//st->nb_frames = 0;
	st->codecpar->codec_id = codec_id;
	st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
	st->codecpar->width = cam.width; //ftl::codecs::getWidth(pkt.definition);
	//if (pkt.flags & ftl::codecs::kFlagStereo) st->codecpar->width *= 2;
	st->codecpar->height = cam.height; //ftl::codecs::getHeight(pkt.definition);
	st->codecpar->format = AV_PIX_FMT_NV12;
	st->codecpar->bit_rate = 4000000;

	//if (pkt.flags & ftl::codecs::kFlagStereo) av_dict_set(&st->metadata, "stereo_mode", "left_right", 0);
	//if (pkt.flags & ftl::codecs::kFlagStereo) av_dict_set(&oc->metadata, "stereo_mode", "1", 0);
	//if (pkt.flags & ftl::codecs::kFlagStereo) av_dict_set_int(&st->metadata, "StereoMode", 1, 0);

    /* put sample parameters */
    //c->bit_rate = 4000000;
    /* resolution must be a multiple of two */
    //c->width = ftl::codecs::getWidth(pkt.definition);
   // c->height = ftl::codecs::getHeight(pkt.definition);
    /* time base: this is the fundamental unit of time (in seconds) in terms
      of which frame timestamps are represented. for fixed-fps content,
       timebase should be 1/framerate and timestamp increments should be
       identically 1. */
    //c->time_base.den = 20;  // Frame rate
    //c->time_base.num = 1;
    //c->gop_size = 10; /* emit one intra frame every twelve frames at most */
    //c->pix_fmt = AV_PIX_FMT_ABGR;

    // some formats want stream headers to be separate
    //if(oc->oformat->flags & AVFMT_GLOBALHEADER)
    //    st->codecpar-> |= CODEC_FLAG_GLOBAL_HEADER;

    return st;
}

static uint32_t make_id(const ftl::codecs::StreamPacket &spkt) {
	return (((spkt.streamID << 8) + spkt.frame_number) << 8) + int(spkt.channel);
}


struct StreamState {
	int64_t first_ts = 100000000000000000ll;
	std::list<std::pair<ftl::codecs::StreamPacket, ftl::codecs::Packet>> packets;
	bool seen_key = false;
	AVStream *stream = nullptr;

	void insert(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		for (auto i = packets.begin(); i != packets.end(); ++i) {
			if (i->first.timestamp > spkt.timestamp) {
				packets.insert(i, std::make_pair(spkt, pkt));
				return;
			}
		}
		packets.push_back(std::make_pair(spkt, pkt));
	}
};


int main(int argc, char **argv) {
    std::string filename;

	auto root = ftl::configure(argc, argv, "player_default");

	std::string outputfile = root->value("out", std::string("output.mkv"));
	std::vector<std::string> paths = *root->get<std::vector<std::string>>("paths");

	if (paths.size() == 0) {
		LOG(ERROR) << "Missing input ftl file.";
		return -1;
	} else {
		filename = paths[paths.size()-1];
	}

	std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
		LOG(ERROR) << "Could not open file: " << filename;
		return -1;
	}

    ftl::codecs::Reader r(f);
    if (!r.begin()) {
		LOG(ERROR) << "Bad ftl file format";
		return -1;
	}

	AVOutputFormat *fmt;
	AVFormatContext *oc;
	StreamState video_st[10];

	int stream_count = 0;
	std::unordered_map<uint32_t, int> mapping;

	// TODO: Remove in newer versions
	av_register_all();

	fmt = av_guess_format(NULL, outputfile.c_str(), NULL);

	if (!fmt) {
        LOG(ERROR) << "Could not find suitable output format";
        return -1;
    }

    /* allocate the output media context */
    oc = avformat_alloc_context();
    if (!oc) {
        LOG(ERROR) << "Memory error";
        return -1;
    }
    oc->oformat = fmt;

	// TODO: Use URL in newer versions
	snprintf(oc->filename, sizeof(oc->filename), "%s", outputfile.c_str());
	//oc->url = (char*)av_malloc(outputfile.size()+1);
	//snprintf(oc->url, outputfile.size()+1, "%s", outputfile.c_str());

	/* open the output file, if needed */
    if (!(fmt->flags & AVFMT_NOFILE)) {
        if (avio_open(&oc->pb, outputfile.c_str(), AVIO_FLAG_WRITE) < 0) {
            LOG(ERROR) << "Could not open output file: " << outputfile;
            return -1;
        }
    }

    LOG(INFO) << "Converting...";

    int current_stream = root->value("stream", 255);
    int current_channel = root->value("channel", -1);

	//bool stream_added[10] = {false};

	// TODO: In future, find a better way to discover number of streams...
	// Read entire file to find all streams before reading again to write data
	bool res = r.read(90000000000000, [&current_stream,&current_channel,&r,&video_st,oc,&mapping,&stream_count](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		if (spkt.channel != Channel::Colour && spkt.channel != Channel::Right) return;

        //if (spkt.channel != static_cast<ftl::codecs::Channel>(current_channel) && current_channel != -1) return;
        //if (spkt.frame_number == current_stream || current_stream == 255) {

            if (pkt.codec != codec_t::HEVC && pkt.codec != codec_t::H264) {
                return;
            }

			if (spkt.frame_number >= 10) return;  // TODO: Allow for more than 10

			//if (video_st[spkt.frame_number][(spkt.channel == Channel::Left) ? 0 : 1] == nullptr) {
			if ((pkt.codec == codec_t::HEVC && ftl::codecs::hevc::isIFrame(pkt.data.data(), pkt.data.size())) ||
					(pkt.codec == codec_t::H264 && ftl::codecs::h264::isIFrame(pkt.data.data(), pkt.data.size()))) {
				if (mapping.count(make_id(spkt)) == 0) {
					int id = stream_count++;

					if (id >= 10) return;				
					
					auto *dec = ftl::codecs::allocateDecoder(pkt);
					if (!dec) return;

					if (spkt.timestamp < video_st[id].first_ts) video_st[id].first_ts = spkt.timestamp;

					cv::cuda::GpuMat m;
					dec->decode(pkt, m);

					ftl::rgbd::Camera cam;
					cam.width = m.cols;
					cam.height = m.rows;
					// Use decoder to get frame size...
					video_st[id].stream = add_video_stream(oc, pkt, cam);

					ftl::codecs::free(dec);

					mapping[make_id(spkt)] = id;
				}
			}
		//}
	});

	r.end();
	f.clear();
	f.seekg(0);
	if (!r.begin()) {
		LOG(ERROR) << "Bad ftl file format";
		return -1;
	}

	av_dump_format(oc, 0, "output.mkv", 1);
	av_dict_set(&oc->metadata, "title", "Future Tech Lab Recording", 0);
	av_dict_set(&oc->metadata, "artist", "University of Turku", 0);

	if (avformat_write_header(oc, NULL) < 0) {
		LOG(ERROR) << "Failed to write stream header";
	}

    res = r.read(90000000000000, [&current_stream,&current_channel,&r,&video_st,oc,&mapping](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
        //if (spkt.channel != static_cast<ftl::codecs::Channel>(current_channel) && current_channel != -1) return;
        //if (spkt.frame_number == current_stream || current_stream == 255) {

            if (pkt.codec != codec_t::HEVC && pkt.codec != codec_t::H264) {
                return;
            }

            //LOG(INFO) << "Reading packet: (" << (int)spkt.streamID << "," << (int)spkt.channel << ") " << (int)pkt.codec << ", " << (int)pkt.definition;

			auto i = mapping.find(make_id(spkt));
			if (i == mapping.end()) return;
			int id = i->second;

			if (!video_st[id].stream) return;

			bool keyframe = false;
			if (pkt.codec == codec_t::HEVC) {
				if (ftl::codecs::hevc::isIFrame(pkt.data.data(), pkt.data.size())) {
					video_st[id].seen_key = true;
					keyframe = true;
				}
			} else if (pkt.codec == codec_t::H264) {
				if (ftl::codecs::h264::isIFrame(pkt.data.data(), pkt.data.size())) {
					video_st[id].seen_key = true;
					keyframe = true;
				}
			}
			if (!video_st[id].seen_key) return;

			//if (spkt.timestamp > last_ts) framecount++;
			//last_ts = spkt.timestamp;

			video_st[id].insert(spkt, pkt);

			if (video_st[id].packets.size() > 5) {
				auto &spkt = video_st[id].packets.front().first;
				auto &pkt = video_st[id].packets.front().second;
				AVPacket avpkt;
				av_init_packet(&avpkt);
				if (keyframe) avpkt.flags |= AV_PKT_FLAG_KEY;
				//avpkt.pts = framecount*50; //spkt.timestamp - r.getStartTime();
				avpkt.pts = spkt.timestamp - video_st[id].first_ts;
				avpkt.dts = avpkt.pts;
				avpkt.stream_index= video_st[id].stream->index;
				avpkt.data= const_cast<uint8_t*>(pkt.data.data());
				avpkt.size= pkt.data.size();
				avpkt.duration = 1;

				//LOG(INFO) << "write frame: " << avpkt.pts << "," << avpkt.stream_index << "," << avpkt.size;

				/* write the compressed frame in the media file */
				auto ret = av_write_frame(oc, &avpkt);
				if (ret != 0) {
					LOG(ERROR) << "Error writing frame: " << ret;
				}

				video_st[id].packets.pop_front();
			}
        //}
    });

	for (int i=0; i<10; ++i) {
		while (video_st[i].packets.size() > 0) {
			auto &spkt = video_st[i].packets.front().first;
			auto &pkt = video_st[i].packets.front().second;
			AVPacket avpkt;
			av_init_packet(&avpkt);
			//if (keyframe) avpkt.flags |= AV_PKT_FLAG_KEY;
			//avpkt.pts = framecount*50; //spkt.timestamp - r.getStartTime();
			avpkt.pts = spkt.timestamp - video_st[i].first_ts;
			avpkt.dts = avpkt.pts;
			avpkt.stream_index= video_st[i].stream->index;
			avpkt.data= const_cast<uint8_t*>(pkt.data.data());
			avpkt.size= pkt.data.size();
			avpkt.duration = 1;

			//LOG(INFO) << "write frame: " << avpkt.pts << "," << avpkt.stream_index << "," << avpkt.size;

			/* write the compressed frame in the media file */
			auto ret = av_write_frame(oc, &avpkt);
			if (ret != 0) {
				LOG(ERROR) << "Error writing frame: " << ret;
			}

			video_st[i].packets.pop_front();	
		}
	}

	av_write_trailer(oc);
	//avcodec_close(video_st->codec);

	for (int i=0; i<10; ++i) {
		if (video_st[i].stream) av_free(video_st[i].stream);
	}

	if (!(fmt->flags & AVFMT_NOFILE)) {
         /* close the output file */
        avio_close(oc->pb);
    }

	av_free(oc);

    if (!res) LOG(INFO) << "Done.";

    r.end();

	ftl::running = false;
	return 0;
}
