#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/codecs/reader.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/codecs/hevc.hpp>

#include <fstream>

#include <Eigen/Eigen>

extern "C" {
 #include <libavformat/avformat.h>
}

using ftl::codecs::codec_t;
using ftl::codecs::Channel;

static AVStream *add_video_stream(AVFormatContext *oc, const ftl::codecs::Packet &pkt)
{
    AVCodecContext *c;
    AVStream *st;

    st = avformat_new_stream(oc, 0);
    if (!st) {
        fprintf(stderr, "Could not alloc stream\n");
        exit(1);
    }

	AVCodecID codec_id;
	switch (pkt.codec) {
	case codec_t::HEVC : codec_id = AV_CODEC_ID_HEVC; break;
	}

    //c = st->codec;
    //c->codec_id = codec_id;
    //c->codec_type = AVMEDIA_TYPE_VIDEO;

	st->time_base.den = 20;
	st->time_base.num = 1;
	//st->id = oc->nb_streams-1;
	//st->nb_frames = 0;
	st->codecpar->codec_id = codec_id;
	st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
	st->codecpar->width = ftl::codecs::getWidth(pkt.definition);
	st->codecpar->height = ftl::codecs::getHeight(pkt.definition);
	st->codecpar->format = AV_PIX_FMT_NV12;
	st->codecpar->bit_rate = 4000000;

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
	AVStream *video_st[10][2] = {nullptr};

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
	snprintf(oc->filename, sizeof(oc->filename), "%s", outputfile.c_str());

	/* open the output file, if needed */
    if (!(fmt->flags & AVFMT_NOFILE)) {
        if (avio_open(&oc->pb, outputfile.c_str(), AVIO_FLAG_WRITE) < 0) {
            LOG(ERROR) << "Could not open output file: " << outputfile;
            return -1;
        }
    }

    LOG(INFO) << "Converting...";

    int current_stream = root->value("stream", 0);
    int current_channel = root->value("channel", -1);

	//bool stream_added[10] = {false};

	// TODO: In future, find a better way to discover number of streams...
	// Read entire file to find all streams before reading again to write data
	bool res = r.read(90000000000000, [&current_stream,&current_channel,&r,&video_st,oc](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
        if (spkt.channel != static_cast<ftl::codecs::Channel>(current_channel) && current_channel != -1) return;
        if (spkt.streamID == current_stream || current_stream == 255) {

            if (pkt.codec != codec_t::HEVC) {
                return;
            }

			if (spkt.streamID >= 10) return;  // TODO: Allow for more than 10

			if (video_st[spkt.streamID][(spkt.channel == Channel::Left) ? 0 : 1] == nullptr) {
				video_st[spkt.streamID][(spkt.channel == Channel::Left) ? 0 : 1] = add_video_stream(oc, pkt);
			}
		}
	});

	r.end();
	f.clear();
	f.seekg(0);
	if (!r.begin()) {
		LOG(ERROR) << "Bad ftl file format";
		return -1;
	}

	av_dump_format(oc, 0, "output.mkv", 1);
	if (avformat_write_header(oc, NULL) < 0) {
		LOG(ERROR) << "Failed to write stream header";
	}

	bool seen_key[10] = {false};

    res = r.read(90000000000000, [&current_stream,&current_channel,&r,&video_st,oc,&seen_key](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
        if (spkt.channel != static_cast<ftl::codecs::Channel>(current_channel) && current_channel != -1) return;
        if (spkt.streamID == current_stream || current_stream == 255) {

            if (pkt.codec != codec_t::HEVC) {
                return;
            }

            //LOG(INFO) << "Reading packet: (" << (int)spkt.streamID << "," << (int)spkt.channel << ") " << (int)pkt.codec << ", " << (int)pkt.definition;

			if (spkt.streamID >= 10) return;  // TODO: Allow for more than 10

			bool keyframe = false;
			if (pkt.codec == codec_t::HEVC) {
				if (ftl::codecs::hevc::isIFrame(pkt.data.data(), pkt.data.size())) {
					seen_key[spkt.streamID] = true;
					keyframe = true;
				}
			}
			if (!seen_key[spkt.streamID]) return;

            AVPacket avpkt;
			av_init_packet(&avpkt);
			if (keyframe) avpkt.flags |= AV_PKT_FLAG_KEY;
			avpkt.pts = spkt.timestamp - r.getStartTime();
			avpkt.dts = avpkt.pts;
			avpkt.stream_index= video_st[spkt.streamID][(spkt.channel == Channel::Left) ? 0 : 1]->index;
			avpkt.data= const_cast<uint8_t*>(pkt.data.data());
			avpkt.size= pkt.data.size();
			avpkt.duration = 1;

			//LOG(INFO) << "write frame: " << avpkt.pts << "," << avpkt.stream_index << "," << avpkt.size;

			/* write the compressed frame in the media file */
			auto ret = av_write_frame(oc, &avpkt);
			if (ret != 0) {
				LOG(ERROR) << "Error writing frame: " << ret;
			}
        }
    });

	av_write_trailer(oc);
	//avcodec_close(video_st->codec);

	for (int i=0; i<10; ++i) {
		if (video_st[i][0]) av_free(video_st[i][0]);
		if (video_st[i][1]) av_free(video_st[i][1]);
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
