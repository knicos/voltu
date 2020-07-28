#include "realsense_source.hpp"
#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/capabilities.hpp>

using ftl::rgbd::detail::RealsenseSource;
using std::string;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;
using ftl::rgbd::Capability;

static std::string get_device_name(const rs2::device& dev) {
	// Each device provides some information on itself, such as name:
	std::string name = "Unknown Device";
	if (dev.supports(RS2_CAMERA_INFO_NAME))
		name = dev.get_info(RS2_CAMERA_INFO_NAME);

	return name;
}

static std::string get_device_serial(const rs2::device& dev) {
	// the serial number of the device:
	std::string sn = "########";
	if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
		sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

	return sn;
}

RealsenseSource::RealsenseSource(ftl::rgbd::Source *host)
        : ftl::rgbd::BaseSourceImpl(host), align_to_depth_(RS2_STREAM_COLOR) {
	capabilities_ = kCapVideo;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGRA8, 30);
    //cfg.enable_stream(RS2_STREAM_DEPTH);
    //cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGRA8);
    //pipe_.start(cfg);
    rs2::pipeline_profile profile = pipe_.start(cfg);
    rs2::device dev = profile.get_device();
	name_ = get_device_name(dev);
	serial_ = get_device_serial(dev);
    rs2_intrinsics intrin = profile.get_stream(rs2_stream::RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

    rs2::depth_sensor ds = dev.query_sensors().front().as<rs2::depth_sensor>();
    scale_ = ds.get_depth_scale();

	LOG(INFO) << "Realsense device: " << name_ << " #" << serial_;

    params_.width = intrin.width;
    params_.height = intrin.height;
    params_.cx = -intrin.ppx;
    params_.cy = -intrin.ppy;
    params_.fx = intrin.fx;
    params_.fy = intrin.fy;
    params_.maxDepth = 3.0;
    params_.minDepth = 0.1;
	params_.doffs = 0.0;
	params_.baseline = 0.055f;  // TODO: Get from device extrinsics

    do_update_params_ = true;

    LOG(INFO) << "Realsense Intrinsics: " << params_.fx << "," << params_.fy << " - " << params_.cx << "," << params_.cy << " - " << params_.width;
}

RealsenseSource::~RealsenseSource() {
	
}

static bool rs_supported = false;
static bool rs_init = false;

bool RealsenseSource::supported() {
	if (rs_init) return rs_supported;
	rs_init = true;

	rs2::context ctx;
	auto devs = ctx.query_devices();
	rs_supported = devs.size() > 0;
	return rs_supported;
}

bool RealsenseSource::capture(int64_t ts) {
	return true;
}

bool RealsenseSource::retrieve(ftl::rgbd::Frame &frame) {
    if (do_update_params_) {
		do_update_params_ = false;
		frame.setLeft() = params_;
		frame.setPose() = Eigen::Matrix4d::Identity();

		auto &meta = frame.create<std::map<std::string,std::string>>(Channel::MetaData);
		meta["name"] = host_->value("name", host_->getID());
		meta["id"] = host_->getID();
		meta["uri"] = host_->value("uri", std::string(""));
		meta["device"] = name_;
		meta["serial"] = serial_;

		if (!frame.has(Channel::Capabilities)) {
			auto &cap = frame.create<std::unordered_set<Capability>>(Channel::Capabilities);
			cap.emplace(Capability::VIDEO);
			cap.emplace(Capability::LIVE);
			cap.emplace(Capability::ACTIVE);
			cap.emplace(Capability::ADJUSTABLE);
		}
	}

    rs2::frameset frames;
	//if (!pipe_.poll_for_frames(&frames)) return false;  //wait_for_frames();

	// TODO: Move to capture function
	try {
		frames = pipe_.wait_for_frames(10);
	} catch (const std::exception &e) {
		return false;
	}

	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    if (host_->value("colour_only", false)) {
        auto cframe = frames.get_color_frame();
        size_t w = cframe.get_width();
        size_t h = cframe.get_height();

        if (params_.width != w) {
            params_.width = w;
            params_.height = h;
            //state_.getLeft() = params_;
        }

        cv::Mat tmp_rgb(cv::Size(w, h), CV_8UC4, (void*)cframe.get_data(), cv::Mat::AUTO_STEP);
        frame.create<GpuMat>(Channel::Colour).upload(tmp_rgb);
    } else {
		auto cframe = frames.get_color_frame(); //first(RS2_STREAM_COLOR);
		size_t w = cframe.get_width();
        size_t h = cframe.get_height();
		cv::Mat wrap_rgb(cv::Size(w, h), CV_8UC4, (void*)cframe.get_data(), cv::Mat::AUTO_STEP);
        frame.create<GpuMat>(Channel::Colour).upload(wrap_rgb, stream_);

        frames = align_to_depth_.process(frames);

        rs2::depth_frame depth = frames.get_depth_frame();
        w = depth.get_width();
        h = depth.get_height();

        cv::Mat wrap_depth(cv::Size((int)w, (int)h), CV_16UC1, (void*)depth.get_data(), depth.get_stride_in_bytes());
		tmp_depth_.upload(wrap_depth, stream_);
        tmp_depth_.convertTo(frame.create<GpuMat>(Channel::Depth), CV_32FC1, scale_, stream_);

		stream_.waitForCompletion();
    }

	return true;
}

bool RealsenseSource::isReady() {
    return true;
}
