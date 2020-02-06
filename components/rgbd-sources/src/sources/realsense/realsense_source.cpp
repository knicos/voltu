#include "realsense_source.hpp"
#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/source.hpp>

using ftl::rgbd::detail::RealsenseSource;
using std::string;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;

RealsenseSource::RealsenseSource(ftl::rgbd::Source *host)
        : ftl::rgbd::detail::Source(host), align_to_depth_(RS2_STREAM_COLOR) {
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
    params_.maxDepth = 3.0;
    params_.minDepth = 0.1;
	params_.doffs = 0.0;

    state_.getLeft() = params_;

    LOG(INFO) << "Realsense Intrinsics: " << params_.fx << "," << params_.fy << " - " << params_.cx << "," << params_.cy << " - " << params_.width;
}

RealsenseSource::~RealsenseSource() {

}

bool RealsenseSource::compute(int n, int b) {
    frame_.reset();
	frame_.setOrigin(&state_);

    rs2::frameset frames;
	if (!pipe_.poll_for_frames(&frames)) return false;  //wait_for_frames();

	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    if (host_->value("colour_only", false)) {
        auto cframe = frames.get_color_frame();
        int w = cframe.get_width();
        int h = cframe.get_height();

        if (params_.width != w) {
            params_.width = w;
            params_.height = h;
            state_.getLeft() = params_;
        }

        cv::Mat tmp_rgb(cv::Size(w, h), CV_8UC4, (void*)cframe.get_data(), cv::Mat::AUTO_STEP);
        frame_.create<GpuMat>(Channel::Colour).upload(tmp_rgb);
    } else {
        frames = align_to_depth_.process(frames);

        rs2::depth_frame depth = frames.get_depth_frame();
        float w = depth.get_width();
        float h = depth.get_height();
        rscolour_ = frames.first(RS2_STREAM_COLOR); //.get_color_frame();

        cv::Mat tmp_depth(cv::Size((int)w, (int)h), CV_16UC1, (void*)depth.get_data(), depth.get_stride_in_bytes());
        tmp_depth.convertTo(tmp_depth, CV_32FC1, scale_);
        frame_.create<GpuMat>(Channel::Depth).upload(tmp_depth);
        cv::Mat tmp_rgb(cv::Size(w, h), CV_8UC4, (void*)rscolour_.get_data(), cv::Mat::AUTO_STEP);
        frame_.create<GpuMat>(Channel::Colour).upload(tmp_rgb);
    }

	host_->notify(timestamp_, frame_);
    return true;
}

bool RealsenseSource::isReady() {
    return true;
}
