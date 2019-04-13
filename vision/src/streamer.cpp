#include <glog/logging.h>
#include <ftl/streamer.hpp>
#include <vector>
#include <zlib.h>
// #include <lz4.h>

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

//static Mat last;

void Streamer::send(const Mat &rgb, const Mat &depth) {
	// Compress the rgb as jpeg.
	vector<unsigned char> rgb_buf;
	cv::imencode(".jpg", rgb, rgb_buf);
	
	Mat d2;
    depth.convertTo(d2, CV_16UC1, 256);
	
	//if (last.rows == 0) d2.copyTo(last);
	
	//Mat ddepth;
	//ddepth = d2 - last;
	
	vector<unsigned char> d_buf;
	/*d_buf.resize(d2.step*d2.rows);
	z_stream defstream;
    defstream.zalloc = Z_NULL;
    defstream.zfree = Z_NULL;
    defstream.opaque = Z_NULL;
    defstream.avail_in = d2.step*d2.rows;
    defstream.next_in = (Bytef *)d2.data; // input char array
    defstream.avail_out = (uInt)d2.step*d2.rows; // size of output
    defstream.next_out = (Bytef *)d_buf.data(); // output char array
    
    deflateInit(&defstream, 4); // Z_DEFAULT_COMPRESSION
    deflate(&defstream, Z_FINISH);
    deflateEnd(&defstream);
    
    d2.copyTo(last);
    
    d_buf.resize(defstream.total_out);*/
    
    // LZ4 Version
    // d_buf.resize(LZ4_compressBound(depth.step*depth.rows));
    // int s = LZ4_compress_default((char*)depth.data, (char*)d_buf.data(), depth.step*depth.rows, d_buf.size());
    // d_buf.resize(s);
    
    //Mat d2;
    //depth.convertTo(d2, CV_16UC1, 256);
    cv::imencode(".png", d2, d_buf);
    LOG(INFO) << "Depth Size = " << ((float)d_buf.size() / (1024.0f*1024.0f));
    
    net_.publish(uri_, rgb_buf, d_buf);
}

