/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#define LOGURU_WITH_STREAMS 1
#include <loguru.hpp>
#include <ftl/config.h>
#include <ftl/configuration.hpp>
#include <ftl/master.hpp>
#include <ftl/threads.hpp>
#include <ftl/codecs/channels.hpp>
#include <ftl/codecs/depth_convert_cuda.hpp>

#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/quality/qualitypsnr.hpp>
#include <ftl/net/universe.hpp>

#include <ftl/streams/filestream.hpp>
#include <ftl/streams/receiver.hpp>
#include <ftl/streams/sender.hpp>
#include <ftl/streams/netstream.hpp>

#include <ftl/operators/colours.hpp>
#include <ftl/operators/mask.hpp>
#include <ftl/operators/segmentation.hpp>
#include <ftl/operators/depth.hpp>

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

using ftl::net::Universe;
using std::string;
using std::vector;
using ftl::config::json_t;
using ftl::codecs::Channel;
using ftl::codecs::codec_t;
using ftl::codecs::definition_t;

using json = nlohmann::json;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

static ftl::rgbd::Generator *createFileGenerator(ftl::Configurable *root, const std::string &filename) {
	ftl::stream::File *stream = ftl::create<ftl::stream::File>(root, "player");
	stream->set("filename", filename);

	ftl::stream::Receiver *gen = ftl::create<ftl::stream::Receiver>(root, "receiver");
	gen->setStream(stream);

	stream->begin();
	stream->select(0, Channel::Colour + Channel::Depth);  // TODO: Choose these elsewhere
	return gen;
}

static void visualizeDepthMap(	const cv::Mat &depth, cv::Mat &out,
								const float max_depth)
{
	DCHECK(max_depth > 0.0);

	depth.convertTo(out, CV_8U, 255.0f / max_depth);
	out = 255 - out;
	//cv::Mat mask = (depth >= max_depth); // TODO (mask for invalid pixels)
	
	applyColorMap(out, out, cv::COLORMAP_JET);
	//out.setTo(cv::Scalar(0), mask);
	//cv::cvtColor(out,out, cv::COLOR_BGR2BGRA);
}

static void run(ftl::Configurable *root) {
	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Master ctrl(root, net);

	net->start();
	net->waitConnections();

	ftl::codecs::Packet pkt;
	pkt.codec = codec_t::HEVC;
	pkt.bitrate = 255;
	pkt.definition = definition_t::Any;
	pkt.flags = ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth;
	pkt.frame_count = 1;

	auto *enc = ftl::codecs::allocateEncoder();
	auto *dec = ftl::codecs::allocateDecoder(pkt);
	cv::cuda::GpuMat gtmp, g_yuv8, g_yuv16;

	double count = 0.0;
	double avgpsnr = 0.0;
	double avgbitrate = 0.0;

	bool toggle_filter = false;
	bool toggle_median = false;
	bool toggle_mask = false;
	bool toggle_bilat = false;

	auto *pre_pipeline_ = ftl::config::create<ftl::operators::Graph>(root, "pre_filters");
	//pre_pipeline_->append<ftl::operators::HFSmoother>("hfnoise");
	pre_pipeline_->append<ftl::operators::ColourChannels>("colour");  // Convert BGR to BGRA
	pre_pipeline_->append<ftl::operators::CrossSupport>("cross");
	auto *maskfilter = pre_pipeline_->append<ftl::operators::DiscontinuityMask>("discontinuity");

	auto *post_filter = ftl::config::create<ftl::operators::Graph>(root, "post_filters");
	//post_filter->append<ftl::operators::CullDiscontinuity>("mask");
	auto *bilat = post_filter->append<ftl::operators::DepthBilateralFilter>("bilateral", Channel::Depth);

	bilat->set("edge_discontinuity", 0.04f);
	bilat->set("max_discontinuity", 0.07f);
	bilat->set("radius", 5);
	bilat->set("iterations", 5);

	ftl::rgbd::Frame oframe;

	maskfilter->set("radius",3);
	//maskfilter->set("threshold", 0.04f);

	// Check paths for FTL files to load.
	auto paths = (*root->get<nlohmann::json>("paths"));
	int i = 0; //groups.size();
	for (auto &x : paths.items()) {
		std::string path = x.value().get<std::string>();
		auto eix = path.find_last_of('.');
		auto ext = path.substr(eix+1);

		// Command line path is ftl file
		if (ext == "ftl") {
			auto *gen = createFileGenerator(root, path);

			gen->onFrameSet([&](ftl::rgbd::FrameSet &fs) {
				fs.id = i;

				if (!fs.frames[0].hasChannel(Channel::Depth)) return true;

				pre_pipeline_->apply(fs,fs,0);

				//fs.frames[0].copyTo(ftl::codecs::Channels<0>(Channel::Colour), oframe);
				fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).copyTo(oframe.create<cv::cuda::GpuMat>(Channel::Colour));
				fs.frames[0].get<cv::cuda::GpuMat>(Channel::Mask).copyTo(oframe.create<cv::cuda::GpuMat>(Channel::Mask));

				auto &gmat = fs.frames[0].get<cv::cuda::GpuMat>(Channel::Depth);
				auto &gtmp = oframe.create<cv::cuda::GpuMat>(Channel::Depth);


				ftl::codecs::Packet pkt;
				pkt.codec = codec_t::HEVC;
				pkt.bitrate = 255;
				pkt.definition = definition_t::Any;
				pkt.flags = ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth;
				pkt.frame_count = 1;

				g_yuv8.create(gmat.size(), CV_8UC4);
				g_yuv16.create(gmat.size(), CV_16UC4);
				gtmp.create(gmat.size(), CV_32F);
				ftl::cuda::depth_to_vuya(gmat, g_yuv8, 16.0f, cv::cuda::Stream::Null());

				if (enc && dec) {
					if (!enc->encode(g_yuv8, pkt)) return true;
					if (!dec->decode(pkt, g_yuv16)) return true;
				} else {
					LOG(ERROR) << "No encoder or decoder";
				}

				double data_rate = double(pkt.data.size()*8*20) / double(1024*1024);
				avgbitrate += data_rate;

				if (toggle_median) {
					cv::Mat tmp;
					g_yuv16.download(tmp);
					cv::medianBlur(tmp, tmp, 5);
					g_yuv16.upload(tmp);
				}

				if (toggle_filter) ftl::cuda::smooth_y(g_yuv16, cv::cuda::Stream::Null());
				ftl::cuda::vuya_to_depth(gtmp, g_yuv16, 16.0f, cv::cuda::Stream::Null());

				if (toggle_bilat) post_filter->apply(oframe,oframe,0);
				
				cv::Mat tmp1, tmp2, tmp3, maski;
				gmat.download(tmp1);
				gtmp.download(tmp2);
				fs.frames[0].get<cv::cuda::GpuMat>(Channel::Mask).download(maski);

				cv::Mat maskd = (maski > 0);
				if (toggle_mask) {
					tmp1.setTo(cv::Scalar(0.0f), maskd);
					tmp2.setTo(cv::Scalar(0.0f), maskd);
				}

				cv::Mat mask = (tmp1 >= 15.0f);
				tmp1.setTo(cv::Scalar(0), mask);

				double psnr = cv::quality::QualityPSNR::compute(tmp1,tmp2,tmp3,16.0f)[0];
				avgpsnr += psnr;
				count += 1.0;

				//cv::absdiff(tmp1,tmp2,tmp1);
				//visualizeDepthMap(tmp1,tmp1,1.0f);
				//tmp1.convertTo(tmp1, CV_8U, 255.0f / 1.0f);

				tmp3.convertTo(tmp3, CV_8U, 255.0f);
				cv::cvtColor(tmp3,tmp3,cv::COLOR_GRAY2BGR);

				//tmp3.setTo(cv::Scalar(0,0,255), maskd);

				cv::putText(tmp3,
						std::string("PSNR ") + std::to_string(psnr) + std::string("db"),
						cv::Point2i(10, 30),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(64, 64, 255), 1);

				cv::putText(tmp3,
						std::string("Bitrate ") + std::to_string(data_rate) + std::string("Mbps"),
						cv::Point2i(10, 50),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(64, 64, 255), 1);

				cv::putText(tmp3,
						std::string("Avg PSNR ") + std::to_string(avgpsnr / count) + std::string("dB"),
						cv::Point2i(10, 70),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(64, 64, 255), 1);

				cv::putText(tmp3,
						std::string("Avg Bitrate ") + std::to_string(avgbitrate / count) + std::string("Mbps"),
						cv::Point2i(10, 90),
						cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(64, 64, 255), 1);

				cv::imshow("Frame", tmp3);

				/*cv::Mat ytmp;
				g_yuv16.download(ytmp);
				cv::Mat planes[4];
				cv::split(ytmp,planes);
				cv::imshow("Y Channel", planes[2]);*/

				visualizeDepthMap(tmp2,tmp2,8.0f);
				cv::imshow("Depth", tmp2);

				auto k = cv::waitKey(1);

				if (k == 'f') {
					toggle_filter = !toggle_filter;
				} else if (k == 'm') {
					toggle_median = !toggle_median;
				} else if (k == 'd') {
					toggle_mask = !toggle_mask;
				} else if (k == 'b') {
					toggle_bilat = !toggle_bilat;
				}

				return true;
			});

			++i;
		}
	}

	LOG(INFO) << "Start timer";
	ftl::timer::start(true);

	LOG(INFO) << "Shutting down...";
	ftl::timer::stop();
	ftl::pool.stop(true);
	ctrl.stop();
	net->shutdown();

	//cudaProfilerStop();

	LOG(INFO) << "Deleting...";

	delete net;

	ftl::config::cleanup();  // Remove any last configurable objects.
	LOG(INFO) << "Done.";
}

int main(int argc, char **argv) {
	run(ftl::configure(argc, argv, "tools_default"));
}
