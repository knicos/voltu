#include "realsense_source.hpp"
#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/source.hpp>

using ftl::rgbd::detail::RealsenseSource;
using std::string;

RealsenseSource::RealsenseSource(ftl::rgbd::Source *host)
        : ftl::rgbd::detail::Source(host), align_to_depth_(RS2_STREAM_DEPTH) {
	capabilities_ = kCapVideo;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGRA8, 30);
    //cfg.enable_stream(RS2_STREAM_DEPTH);
    //cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGRA8);
    //pipe_.start(cfg);
    rs2::pipeline_profile profile = pipe_.start(cfg);
    rs2::device dev = profile.get_device();
    rs2_intrinsics intrin = profile.get_stream(rs2_stream::RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

    rs2::depth_sensor ds = dev.query_sensors().front().as<rs2::depth_sensor>();
    scale_ = ds.get_depth_scale();
    LOG(INFO) << "RS Scale = " << scale_;

    params_.width = intrin.width;
    params_.height = intrin.height;
    params_.cx = -intrin.ppx;
    params_.cy = -intrin.ppy;
    params_.fx = intrin.fx;
    params_.fy = intrin.fy;
    params_.maxDepth = 11.0;
    params_.minDepth = 0.1;

    //LOG(INFO) << "Realsense Intrinsics: " << params_.fx << "," << params_.fy << " - " << params_.cx << "," << params_.cy << " - " << params_.width;
}

RealsenseSource::~RealsenseSource() {

}

bool RealsenseSource::grab(int n, int b) {
    rs2::frameset frames = pipe_.wait_for_frames();
    //rs2::align align(RS2_STREAM_DEPTH);
    //frames = align_to_depth_.process(frames); //align_to_depth_.process(frames);

    rs2::depth_frame depth = frames.get_depth_frame();
    float w = depth.get_width();
    float h = depth.get_height();
    rscolour_ = frames.first(RS2_STREAM_COLOR); //.get_color_frame();

    cv::Mat tmp(cv::Size((int)w, (int)h), CV_16UC1, (void*)depth.get_data(), depth.get_stride_in_bytes());
    tmp.convertTo(depth_, CV_32FC1, scale_);
    rgb_ = cv::Mat(cv::Size(w, h), CV_8UC4, (void*)rscolour_.get_data(), cv::Mat::AUTO_STEP);
    return true;
}

bool RealsenseSource::isReady() {
    return true;
}
