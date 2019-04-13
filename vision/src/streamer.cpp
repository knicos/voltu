#include <glog/logging.h>
#include <ftl/streamer.hpp>
#include <vector>
#include <zlib.h>

using ftl::Streamer;
using ftl::net::Universe;
using cv::Mat;
using nlohmann::json;
using std::string;
using std::vector;

Streamer::Streamer(Universe &net, json &config) : net_(net), config_(config) {
	uri_ = string("ftl://utu.fi/")+(string)config["name"]+string("/rgb-d");
	net.createResource(uri_);
}

Streamer::~Streamer() {

}

void Streamer::send(const Mat &rgb, const Mat &depth) {
	// Compress the rgb as jpeg.
	vector<unsigned char> rgb_buf;
	cv::imencode(".jpg", rgb, rgb_buf);
	
	vector<unsigned char> d_buf;
	d_buf.resize(depth.step*depth.rows);
	z_stream defstream;
    defstream.zalloc = Z_NULL;
    defstream.zfree = Z_NULL;
    defstream.opaque = Z_NULL;
    defstream.avail_in = depth.step*depth.rows;
    defstream.next_in = (Bytef *)depth.data; // input char array
    defstream.avail_out = (uInt)depth.step*depth.rows; // size of output
    defstream.next_out = (Bytef *)d_buf.data(); // output char array
    
    deflateInit(&defstream, Z_DEFAULT_COMPRESSION);
    deflate(&defstream, Z_FINISH);
    deflateEnd(&defstream);
    
    d_buf.resize(defstream.total_out);
    //Mat d2;
    //depth.convertTo(d2, CV_16UC1, 256);
    //cv::imencode(".png", depth, d_buf);
    LOG(INFO) << "Depth Size = " << ((float)d_buf.size() / (1024.0f*1024.0f));
    
    net_.publish(uri_, rgb_buf, d_buf);
}

