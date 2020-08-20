#include "pylon.hpp"

#include "rectification.hpp"

#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/profiler.hpp>
#include <ftl/rgbd/frame.hpp>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include <opencv2/imgproc.hpp>

#include <nlohmann/json.hpp>

using ftl::rgbd::detail::StereoRectification;
using ftl::rgbd::detail::PylonDevice;
using std::string;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;
using cv::Mat;
using namespace Pylon;

PylonDevice::PylonDevice(nlohmann::json &config)
		: ftl::rgbd::detail::Device(config), ready_(false), lcam_(nullptr), rcam_(nullptr),
		interpolation_(cv::INTER_CUBIC) {

	auto &inst = CTlFactory::GetInstance();

	Pylon::DeviceInfoList_t devices;
	inst.EnumerateDevices(devices);

	int dev_left_num = -1;
	std::string dev_left;

	if (getConfig()["device_left"].is_number()) {
		dev_left = std::to_string(value("device_left",0));
	} else {
		dev_left = value("device_left", std::string("default"));
	}

	if (devices.size() == 0) {
		LOG(ERROR) << "No Pylon devices attached";
		return;
	} else {
		int i=0;
		for (auto d : devices) {
			if (std::string(d.GetSerialNumber()) == dev_left) {
				dev_left_num = i;
			}

			if (dev_left_num == i) {
				LOG(INFO) << " - found Pylon device - " << d.GetSerialNumber() << " (" << d.GetModelName() << ") [primary]";
			} else {
				LOG(INFO) << " - found Pylon device - " << d.GetSerialNumber() << " (" << d.GetModelName() << ")";
			}

			++i;
		}
	}

	if (dev_left_num == -1) dev_left_num = 0;

	name_ = devices[dev_left_num].GetModelName();
	serial_ = devices[dev_left_num].GetSerialNumber();

	try {
		lcam_ = new CBaslerUniversalInstantCamera( CTlFactory::GetInstance().CreateDevice(devices[dev_left_num]));
		lcam_->RegisterConfiguration( new Pylon::CSoftwareTriggerConfiguration, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);
		lcam_->Open();

		if (devices.size() >= 2) {
			int dev_right = (dev_left_num == 0) ? 1 : 0;
			rcam_ = new CBaslerUniversalInstantCamera( CTlFactory::GetInstance().CreateDevice(devices[dev_right]));
			rcam_->RegisterConfiguration( new Pylon::CSoftwareTriggerConfiguration, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);
			rcam_->Open();
		}

		_configureCamera(lcam_);
		if (rcam_) _configureCamera(rcam_);

		lcam_->StartGrabbing( Pylon::GrabStrategy_OneByOne);
		if (rcam_) rcam_->StartGrabbing( Pylon::GrabStrategy_OneByOne);

		if (rcam_) rcam_->WaitForFrameTriggerReady( 300, Pylon::TimeoutHandling_ThrowException);
		lcam_->WaitForFrameTriggerReady( 300, Pylon::TimeoutHandling_ThrowException);

		ready_ = true;
	} catch (const Pylon::GenericException &e) {
		// Error handling.
		LOG(ERROR) << "Pylon: An exception occurred - " << e.GetDescription();
	}

	// Choose a good default depth res
	//width_ = value("depth_width", std::min(1280u,fullwidth_)) & 0xFFFe;
	//float aspect = float(fullheight_) / float(fullwidth_);
	//height_ = value("depth_height", std::min(uint32_t(aspect*float(width_)), fullheight_)) & 0xFFFe;

	//LOG(INFO) << "Depth resolution: " << width_ << "x" << height_;

	// Allocate page locked host memory for fast GPU transfer
	left_hm_ = cv::cuda::HostMem(fullheight_, fullwidth_, CV_8UC4);
	right_hm_ = cv::cuda::HostMem(fullheight_, fullwidth_, CV_8UC4);
	//hres_hm_ = cv::cuda::HostMem(fullheight_, fullwidth_, CV_8UC4);
	//rtmp_.create(fullheight_, fullwidth_, CV_8UC4);

	on("exposure", [this]() {
		if (lcam_->GetDeviceInfo().GetModelName() != "Emulation") {
			lcam_->ExposureTime.SetValue(value("exposure", 24000.0f));  // Exposure time in microseconds
		}
		if (rcam_ && rcam_->GetDeviceInfo().GetModelName() != "Emulation") {
			rcam_->ExposureTime.SetValue(value("exposure", 24000.0f));  // Exposure time in microseconds
		}
	});

	on("buffer_size", buffer_size_, 1);

	interpolation_ = value("inter_cubic", true) ? cv::INTER_CUBIC : cv::INTER_LINEAR;
	on("inter_cubic", [this](){
		interpolation_ = value("inter_cubic", true) ?
			cv::INTER_CUBIC : cv::INTER_LINEAR;
	});

	monitor_ = true;
	temperature_monitor_ = ftl::timer::add(ftl::timer::timerlevel_t::kTimerIdle1, 10.0, [this](int64_t ts) {
		float temperature = (rcam_) ? std::max(lcam_->DeviceTemperature(), rcam_->DeviceTemperature()) : lcam_->DeviceTemperature();

		LOG_IF(WARNING, temperature > 53.0)
			<< "Camera temperature over 50C (value: " << temperature << ")";

		// TODO: check actual temperature status.
		if (temperature > 65.0) {
			LOG(FATAL) << "Cameras are overheating";
		}

		return true;
	});
}

PylonDevice::~PylonDevice() {
	monitor_ = false;
	temperature_monitor_.cancel();

	lcam_->Close();
	rcam_->Close();
}

static std::vector<ftl::rgbd::detail::DeviceDetails> pylon_devices;
static bool pylon_dev_init = false;

std::vector<ftl::rgbd::detail::DeviceDetails> PylonDevice::listDevices() {
	if (pylon_dev_init) return pylon_devices;
	pylon_dev_init = true;

	auto &inst = CTlFactory::GetInstance();

	Pylon::DeviceInfoList_t devices;
	inst.EnumerateDevices(devices);

	std::vector<ftl::rgbd::detail::DeviceDetails> results;

	int count=0;
	for (auto d : devices) {
		//LOG(INFO) << " - found Pylon device - " << d.GetFullName() << "(" << d.GetModelName() << ")";
		auto &r = results.emplace_back();
		r.id = count++;
		r.name = d.GetModelName();
		r.maxheight = 0;
		r.maxwidth = 0;
	}

	pylon_devices = results;
	return results;
}

void PylonDevice::_configureCamera(CBaslerUniversalInstantCamera *cam) {
	// Get the camera control object.
	GenApi::INodeMap& nodemap = cam->GetNodeMap();
	// Get the parameters for setting the image area of interest (Image AOI).
	CIntegerParameter width(nodemap, "Width");
	CIntegerParameter height(nodemap, "Height");
	CIntegerParameter offsetX(nodemap, "OffsetX");
	CIntegerParameter offsetY(nodemap, "OffsetY");

	fullwidth_ = width.GetValue();
	fullheight_ = height.GetValue();

	LOG(INFO) << "Camera resolution = " << fullwidth_ << "x" << fullheight_;

	// Set the pixel data format.
	CEnumParameter format(nodemap, "PixelFormat");
	LOG(INFO) << "Camera format: " << format.GetValue();

	if (format.CanSetValue("BayerBG8")) {  // YCbCr422_8
		format.SetValue("BayerBG8");
	} else {
		LOG(WARNING) << "Could not change pixel format";
	}

	if (cam->GetDeviceInfo().GetModelName() != "Emulation") {
		// Emulated device throws exception with these
		cam->ExposureTime.SetValue(value("exposure", 24000.0f));  // Exposure time in microseconds
		cam->AutoTargetBrightness.SetValue(0.3);
		cam->LightSourcePreset.SetValue(Basler_UniversalCameraParams::LightSourcePreset_Tungsten2800K);  // White balance option
		cam->BalanceWhiteAuto.SetValue(Basler_UniversalCameraParams::BalanceWhiteAuto_Once);
		cam->GainAuto.SetValue(Basler_UniversalCameraParams::GainAuto_Once);
	}
}

bool PylonDevice::grab() {
	if (!isReady()) return false;

	//int dev;
	//cudaGetDevice(&dev);
	//LOG(INFO) << "Current cuda device = " << dev;

	try {
		FTL_Profile("Frame Capture", 0.001);
		if (rcam_) rcam_->WaitForFrameTriggerReady( 0, Pylon::TimeoutHandling_ThrowException);
		lcam_->WaitForFrameTriggerReady( 0, Pylon::TimeoutHandling_ThrowException);

		lcam_->ExecuteSoftwareTrigger();
		if (rcam_) rcam_->ExecuteSoftwareTrigger();
	} catch (const GenericException &e) {
		LOG(ERROR) << "Pylon: Trigger exception - " << e.GetDescription();
		return false;
	}

	return true;
}

bool PylonDevice::_retrieveFrames(Pylon::CGrabResultPtr &result, Pylon::CBaslerUniversalInstantCamera *cam) {
	do {
		if (cam->RetrieveResult(0, result, Pylon::TimeoutHandling_Return)) {
			if (!result->GrabSucceeded()) {
				LOG(ERROR) << "Retrieve failed " << result->GetErrorDescription();
			}
		} else {
			LOG(ERROR) << "Pylon frame missing";
			return false;
		}
	} while (!result->GrabSucceeded());
	return true;
}

bool PylonDevice::get(ftl::rgbd::Frame &frame, StereoRectification *c, cv::cuda::Stream &stream) {
	if (!isReady()) return false;

	Mat l, r;

	// Use page locked memory
	l = left_hm_.createMatHeader();
	r = right_hm_.createMatHeader();

	if (isStereo()) {
		auto lcount = lcam_->NumReadyBuffers.GetValue();
		auto rcount = rcam_->NumReadyBuffers.GetValue();

		/*if (left_fail_) {
			left_fail_ = 0;
			Pylon::CGrabResultPtr tmp_result;
			lcam_->RetrieveResult(0, tmp_result, Pylon::TimeoutHandling_Return);
		}
		if (rcam_ && right_fail_) {
			right_fail_ = 0;
			Pylon::CGrabResultPtr tmp_result;
			rcam_->RetrieveResult(0, tmp_result, Pylon::TimeoutHandling_Return);
		}*/

		if (rcount < buffer_size_ || lcount < buffer_size_) {
			LOG(WARNING) << "Retrieve failed for L+R";
			return false;
		}

		if (rcount > buffer_size_ && lcount > buffer_size_) {
			LOG(WARNING) << "Pylon buffer latency problem : " << lcount << " vs " << rcount << " frames";
			Pylon::CGrabResultPtr tmp_result;
			//lcam_->RetrieveResult(0, tmp_result, Pylon::TimeoutHandling_Return);
			//rcam_->RetrieveResult(0, tmp_result, Pylon::TimeoutHandling_Return);
			_retrieveFrames(tmp_result, lcam_);
			_retrieveFrames(tmp_result, rcam_);
		} else if (rcount > buffer_size_) LOG(ERROR) << "Buffers (R) out of sync by " << rcount;
		else if (lcount > buffer_size_) LOG(ERROR) << "Buffers (L) out of sync by " << lcount;
	} else {
		if (lcam_->NumReadyBuffers.GetValue() == 0) {
			LOG(INFO) << "Retrieve failed for L";
			return false;
		}
	}

	try {
		FTL_Profile("Frame Retrieve", 0.005);
		bool res_r = false;

		if (rcam_) {
			Pylon::CGrabResultPtr result_right;

			if (_retrieveFrames(result_right, rcam_)) {

				cv::Mat wrap_right(
				result_right->GetHeight(),
				result_right->GetWidth(),
				CV_8UC1,
				(uint8_t*)result_right->GetBuffer());

				{
					FTL_Profile("Bayer Colour (R)", 0.005);
					cv::cvtColor(wrap_right, rtmp_, cv::COLOR_BayerRG2BGRA);
				}

				{
					FTL_Profile("Rectify (R)", 0.005);
					c->rectify(rtmp_, r, Channel::Right);
				}

				auto& f_right = frame.create<ftl::rgbd::VideoFrame>(Channel::Right);
				cv::cuda::GpuMat& r_out = f_right.createGPU();
				cv::Mat &r_host = f_right.setCPU();

				r_out.upload(r, stream);
				r.copyTo(r_host);
				res_r = true;
			}
		}

		Pylon::CGrabResultPtr result_left;

		if (!_retrieveFrames(result_left, lcam_)) {
			return false;
		}

		cv::Mat wrap_left(
			result_left->GetHeight(),
			result_left->GetWidth(),
			CV_8UC1,
			(uint8_t*)result_left->GetBuffer());

		{
			FTL_Profile("Bayer Colour (L)", 0.005);
			if (isStereo()) cv::cvtColor(wrap_left, ltmp_, cv::COLOR_BayerRG2BGRA);
			else cv::cvtColor(wrap_left, l, cv::COLOR_BayerRG2BGRA);
		}

		{
			FTL_Profile("Rectify (L)", 0.005);
			if (isStereo()) {
				c->rectify(ltmp_, l, Channel::Left);
			}
		}

		auto& f_left = frame.create<ftl::rgbd::VideoFrame>(Channel::Left);
		cv::cuda::GpuMat& l_out = f_left.createGPU();
		cv::Mat &l_host = f_left.setCPU();

		l_out.upload(l, stream);
		l.copyTo(l_host);

		if (rcam_) {
			if (!res_r) return false;
		}

	} catch (const GenericException &e) {
		LOG(ERROR) << "Pylon: An exception occurred - " << e.GetDescription();
	}

	return true;
}

bool PylonDevice::isReady() const {
	return lcam_ && lcam_->IsOpen();
}

void PylonDevice::populateMeta(std::map<std::string,std::string> &meta) const {
	meta["device"] = name_;
	meta["serial"] = serial_;
}

