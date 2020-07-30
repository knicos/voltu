/*
 * Copyright 2019 Nicolas Pope
 */

#include <loguru.hpp>

#include <string>
#include <chrono>
#include <ftl/threads.hpp>
#include <ftl/profiler.hpp>
#include <ftl/rgbd/frame.hpp>

#include "opencv.hpp"
#include "rectification.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto.hpp>
#include <opencv2/imgcodecs.hpp>

#include <ftl/timer.hpp>

#ifndef WIN32
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#else
#include <mfapi.h>
#include <mfidl.h>
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfuuid.lib")
#endif

using ftl::rgbd::detail::OpenCVDevice;
using ftl::rgbd::detail::StereoRectification;
using ftl::codecs::Channel;
using cv::Mat;
using cv::VideoCapture;
using cv::Rect;
using std::string;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

OpenCVDevice::OpenCVDevice(nlohmann::json &config, bool stereo)
		: ftl::rgbd::detail::Device(config), timestamp_(0.0),
		interpolation_(cv::INTER_CUBIC) {

	std::vector<ftl::rgbd::detail::DeviceDetails> devices_ = getDevices();

	int device_left = 0;
	int device_right = -1;

	LOG(INFO) << "Found " << devices_.size() << " cameras";

	if (Configurable::get<std::string>("device_left")) {
		for (auto &d : devices_) {
			if (d.name.find(*Configurable::get<std::string>("device_left")) != std::string::npos) {
				device_left = d.id;
				LOG(INFO) << "Device left = " << device_left;
				break;
			}
		}
	} else {
		device_left = value("device_left", (devices_.size() > 0) ? devices_[0].id : 0);
	}

	if (Configurable::get<std::string>("device_right")) {
		for (auto &d : devices_) {
			if (d.name.find(*Configurable::get<std::string>("device_right")) != std::string::npos) {
				if (d.id == device_left) continue;
				device_right = d.id;
				break;
			}
		}
	} else {
		device_right = value("device_right", (devices_.size() > 1) ? devices_[1].id : 1);
	}

	nostereo_ = value("nostereo", !stereo);

	if (device_left < 0) {
		LOG(ERROR) << "No available cameras";
		return;
	}

	dev_ix_left_ = device_left;
	dev_ix_right_ = device_right;

	// Use cameras
	camera_a_ = new VideoCapture;
	LOG(INFO) << "Cameras check... ";
	camera_a_->open(device_left);

	if (!nostereo_ && device_right >= 0) {
		camera_b_ = new VideoCapture(device_right);
	} else {
		camera_b_ = nullptr;
	}

	if (!camera_a_->isOpened()) {
		delete camera_a_;
		if (camera_b_) delete camera_b_;
		camera_a_ = nullptr;
		camera_b_ = nullptr;
		LOG(FATAL) << "No cameras found";
		return;
	}

	if (!camera_b_ || !camera_b_->isOpened()) {
		if (camera_b_) delete camera_b_;
		camera_b_ = nullptr;
		stereo_ = false;
		LOG(WARNING) << "Not able to find second camera for stereo";
	}
	else {
		camera_b_->set(cv::CAP_PROP_FRAME_WIDTH, value("width", 1280));
		camera_b_->set(cv::CAP_PROP_FRAME_HEIGHT, value("height", 720));
		camera_b_->set(cv::CAP_PROP_FPS, 1000 / ftl::timer::getInterval());
		//camera_b_->set(cv::CAP_PROP_BUFFERSIZE, 0);  // Has no effect

		stereo_ = true;
	}

	LOG(INFO) << "Video backend: " << camera_a_->getBackendName();
	LOG(INFO) << "Video defaults: " << camera_a_->get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camera_a_->get(cv::CAP_PROP_FRAME_HEIGHT) << " @ " << camera_a_->get(cv::CAP_PROP_FPS);

	camera_a_->set(cv::CAP_PROP_FRAME_WIDTH, value("width", 1280));
	camera_a_->set(cv::CAP_PROP_FRAME_HEIGHT, value("height", 720));
	camera_a_->set(cv::CAP_PROP_FPS, 1000 / ftl::timer::getInterval());
	//camera_a_->set(cv::CAP_PROP_BUFFERSIZE, 0);  // Has no effect

	Mat frame;
	if (!camera_a_->grab()) LOG(ERROR) << "Could not grab a video frame";
	camera_a_->retrieve(frame);
	LOG(INFO) << "Video size : " << frame.cols << "x" << frame.rows;
	width_ = frame.cols;
	height_ = frame.rows;

	dwidth_ = value("depth_width", width_);
	float aspect = float(height_) / float(width_);
	dheight_ = value("depth_height", std::min(uint32_t(aspect*float(dwidth_)), height_)) & 0xFFFe;

	// Allocate page locked host memory for fast GPU transfer
	left_hm_ = cv::cuda::HostMem(dheight_, dwidth_, CV_8UC4);
	right_hm_ = cv::cuda::HostMem(dheight_, dwidth_, CV_8UC4);
	hres_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC4);

	interpolation_ = value("inter_cubic", true) ? cv::INTER_CUBIC : cv::INTER_LINEAR;
	on("inter_cubic", [this](){
		interpolation_ = value("inter_cubic_", true) ?
			cv::INTER_CUBIC : cv::INTER_LINEAR;
	});
}

OpenCVDevice::~OpenCVDevice() {

}

static std::vector<ftl::rgbd::detail::DeviceDetails> opencv_devices;
static bool opencv_dev_init = false;

std::vector<ftl::rgbd::detail::DeviceDetails> OpenCVDevice::getDevices() {
	if (opencv_dev_init) return opencv_devices;
	opencv_dev_init = true;

	std::vector<ftl::rgbd::detail::DeviceDetails> devices;

#ifdef WIN32
	UINT32 count = 0;

	IMFAttributes *pConfig = NULL;
	IMFActivate **ppDevices = NULL;

	// Create an attribute store to hold the search criteria.
	HRESULT hr = MFCreateAttributes(&pConfig, 1);

	// Request video capture devices.
	if (SUCCEEDED(hr))
	{
		hr = pConfig->SetGUID(
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
		);
	}

	// Enumerate the devices,
	if (SUCCEEDED(hr))
	{
		hr = MFEnumDeviceSources(pConfig, &ppDevices, &count);
	}

	// Create a media source for the first device in the list.
	if (SUCCEEDED(hr))
	{
		if (count > 0)
		{
			for (int i = 0; i < count; ++i) {
				HRESULT hr = S_OK;
				WCHAR *szFriendlyName = NULL;

				// Try to get the display name.
				UINT32 cchName;
				hr = ppDevices[i]->GetAllocatedString(
					MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
					&szFriendlyName, &cchName);

				char temp[100];
				size_t size;
				wcstombs_s(&size, temp, 100, szFriendlyName, _TRUNCATE);

				if (SUCCEEDED(hr))
				{
					LOG(INFO) << " -- " << temp;
					devices.push_back({
						std::string((const char*)temp),
						i,
						0,
						0
					});
				}
				CoTaskMemFree(szFriendlyName);
			}
		}
		else
		{

		}
	}

	for (DWORD i = 0; i < count; i++)
	{
		ppDevices[i]->Release();
	}
	CoTaskMemFree(ppDevices);
#else

	int fd;
	v4l2_capability video_cap;
	v4l2_frmsizeenum video_fsize;

	LOG(INFO) << "Video Devices:";

	for (int i=0; i<10; ++i) {
		std::string path = "/dev/video";
		path += std::to_string(i);

		if ((fd = open(path.c_str(), O_RDONLY)) == -1) {
			break;
		}

		if(ioctl(fd, VIDIOC_QUERYCAP, &video_cap) == -1) {
			LOG(WARNING) << "Can't get video capabilities";
			continue;
		}// else {

		// Get some formats
		v4l2_fmtdesc pixfmt;
		pixfmt.index = 0;
		pixfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		while (ioctl(fd, VIDIOC_ENUM_FMT, &pixfmt) == 0) {
			LOG(INFO) << " -- -- format = " << pixfmt.description << " code = " << ((char*)&pixfmt.pixelformat)[0] << ((char*)&pixfmt.pixelformat)[1] << ((char*)&pixfmt.pixelformat)[2] << ((char*)&pixfmt.pixelformat)[3];
			pixfmt.index++;
		}

		memset(&video_fsize, 0, sizeof(video_fsize));
		video_fsize.index = 0;
		video_fsize.pixel_format = v4l2_fourcc('Y','U','Y','V');

		size_t maxwidth = 0;
		size_t maxheight = 0;

		while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &video_fsize) == 0) {
			maxwidth = max(maxwidth, (video_fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) ? video_fsize.discrete.width : video_fsize.stepwise.max_width);
			maxheight = max(maxheight, (video_fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) ? video_fsize.discrete.height : video_fsize.stepwise.max_height);
			video_fsize.index++;
		}

			//printf("Name:\t\t '%s'\n", video_cap.name);
			//printf("Minimum size:\t%d x %d\n", video_cap.minwidth, video_cap.minheight);
			//printf("Maximum size:\t%d x %d\n", video_cap.maxwidth, video_cap.maxheight);

		if (maxwidth > 0 && maxheight > 0) {
			devices.push_back({
				std::string((const char*)video_cap.card),
				i,
				maxwidth,
				maxheight
			});
		}

		LOG(INFO) << " -- " << video_cap.card << " (" << maxwidth << "x" << maxheight << ")";
		//}

		/*if(ioctl(fd, VIDIOCGWIN, &video_win) == -1)
			perror("cam_info: Can't get window information");
		else
			printf("Current size:\t%d x %d\n", video_win.width, video_win.height);

		if(ioctl(fd, VIDIOCGPICT, &video_pic) == -1)
			perror("cam_info: Can't get picture information");
		else
			printf("Current depth:\t%d\n", video_pic.depth);*/

		close(fd);
	}

#endif

	opencv_devices = devices;
	return devices;
}


bool OpenCVDevice::grab() {
	if (!camera_a_) return false;

	if (camera_b_) {
		if (!camera_a_->grab()) {
			LOG(ERROR) << "Unable to grab from camera A";
			return false;
		}
		if (camera_b_ && !camera_b_->grab()) {
			LOG(ERROR) << "Unable to grab from camera B";
			return false;
		}
	}

	return true;
}

bool OpenCVDevice::get(ftl::rgbd::Frame &frame, cv::cuda::GpuMat &l_out, cv::cuda::GpuMat &r_out,
	cv::cuda::GpuMat &l_hres_out, cv::Mat &r_hres_out, StereoRectification *c, cv::cuda::Stream &stream) {

	Mat l, r ,hres;

	// Use page locked memory
	l = left_hm_.createMatHeader();
	r = right_hm_.createMatHeader();
	hres = hres_hm_.createMatHeader();

	Mat &lfull = (!hasHigherRes()) ? l : hres;
	Mat &rfull = (!hasHigherRes()) ? r : rtmp_;

	if (!camera_a_) return false;

	//std::future<bool> future_b;
	if (camera_b_) {
		//future_b = std::move(ftl::pool.push([this,&rfull,&r,c,&r_out,&r_hres_out,&stream](int id) {
			if (!camera_b_->retrieve(frame_r_)) {
				LOG(ERROR) << "Unable to read frame from camera B";
				return false;
			} else {
				cv::cvtColor(frame_r_, rtmp2_, cv::COLOR_BGR2BGRA);

				//if (stereo_) {
					c->rectify(rtmp2_, rfull, Channel::Right);

					if (hasHigherRes()) {
						// TODO: Use threads?
						cv::resize(rfull, r, r.size(), 0.0, 0.0, interpolation_);
						r_hres_out = rfull;
					}
					else {
						r_hres_out = Mat();
					}
				//}

				r_out.upload(r, stream);
			}
			//return true;
		//}));
	}

	if (camera_b_) {
		//FTL_Profile("Camera Retrieve", 0.01);
		// TODO: Use threads here?
		if (!camera_a_->retrieve(frame_l_)) {
			LOG(ERROR) << "Unable to read frame from camera A";
			return false;
		}

		/*if (camera_b_ && !camera_b_->retrieve(rfull)) {
			LOG(ERROR) << "Unable to read frame from camera B";
			return false;
		}*/
	} else {
		if (!camera_a_->read(frame_l_)) {
			LOG(ERROR) << "Unable to read frame from camera A";
			return false;
		}
	}

	if (stereo_) {
		cv::cvtColor(frame_l_, ltmp_, cv::COLOR_BGR2BGRA);
		//FTL_Profile("Rectification", 0.01);
		//c->rectifyStereo(lfull, rfull);
		c->rectify(ltmp_, lfull, Channel::Left);

		// Need to resize
		//if (hasHigherRes()) {
			// TODO: Use threads?
		//	cv::resize(rfull, r, r.size(), 0.0, 0.0, interpolation_);
		//}
	} else {
		cv::cvtColor(frame_l_, lfull, cv::COLOR_BGR2BGRA);
	}

	if (hasHigherRes()) {
		//FTL_Profile("Frame Resize", 0.01);
		cv::resize(lfull, l, l.size(), 0.0, 0.0, interpolation_);
		l_hres_out.upload(hres, stream);
	} else {
		l_hres_out = cv::cuda::GpuMat();
	}

	{
		//FTL_Profile("Upload", 0.05);
		l_out.upload(l, stream);
	}
	//r_out.upload(r, stream);

	if (!frame.hasChannel(Channel::Thumbnail)) {
		cv::Mat thumb;
		cv::resize(l, thumb, cv::Size(320,240));
		auto &thumbdata = frame.create<std::vector<uint8_t>>(Channel::Thumbnail);
		std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
		cv::imencode(".jpg", thumb, thumbdata, params);
	}

	//if (camera_b_) {
		//FTL_Profile("WaitCamB", 0.05);
		//future_b.wait();
	//}

	return true;
}

double OpenCVDevice::getTimestamp() const {
	return timestamp_;
}

bool OpenCVDevice::isStereo() const {
	return stereo_ && !nostereo_;
}

void OpenCVDevice::populateMeta(std::map<std::string,std::string> &meta) const {
	if (dev_ix_left_ >= 0 && dev_ix_left_ < static_cast<int>(opencv_devices.size())) {
		meta["device"] = opencv_devices[dev_ix_left_].name;
	}
}

