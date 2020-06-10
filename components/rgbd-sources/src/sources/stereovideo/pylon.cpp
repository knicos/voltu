#include "pylon.hpp"

#include "calibrate.hpp"
#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/profiler.hpp>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include <opencv2/imgproc.hpp>

using ftl::rgbd::detail::PylonDevice;
using std::string;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;
using cv::Mat;
using namespace Pylon;

PylonDevice::PylonDevice(nlohmann::json &config)
        : ftl::rgbd::detail::Device(config), ready_(false), lcam_(nullptr), rcam_(nullptr) {

	auto &inst = CTlFactory::GetInstance();

	Pylon::DeviceInfoList_t devices;
	inst.EnumerateDevices(devices);

	if (devices.size() == 0) {
		LOG(ERROR) << "No Pylon devices attached";
		return;
	} else {
		for (auto d : devices) {
			LOG(INFO) << " - found Pylon device - " << d.GetFullName() << "(" << d.GetModelName() << ")";
		}
	}

	try {
    	lcam_ = new CBaslerUniversalInstantCamera( CTlFactory::GetInstance().CreateDevice(devices[0]));
		lcam_->RegisterConfiguration( new Pylon::CSoftwareTriggerConfiguration, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);
		lcam_->Open();

		if (devices.size() >= 2) {
			rcam_ = new CBaslerUniversalInstantCamera( CTlFactory::GetInstance().CreateDevice(devices[1]));
			rcam_->RegisterConfiguration( new Pylon::CSoftwareTriggerConfiguration, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);
			rcam_->Open();
		}

		_configureCamera(lcam_);
		if (rcam_) _configureCamera(rcam_);

		lcam_->StartGrabbing( Pylon::GrabStrategy_OneByOne);
		if (rcam_) rcam_->StartGrabbing( Pylon::GrabStrategy_OneByOne);

		ready_ = true;
	} catch (const Pylon::GenericException &e) {
		// Error handling.
        LOG(ERROR) << "Pylon: An exception occurred - " << e.GetDescription();
	}

	// Choose a good default depth res
	width_ = value("depth_width", std::min(1280u,fullwidth_)) & 0xFFFe;
	float aspect = float(fullheight_) / float(fullwidth_);
	height_ = value("depth_height", std::min(uint32_t(aspect*float(width_)), fullheight_)) & 0xFFFe;

	LOG(INFO) << "Depth resolution: " << width_ << "x" << height_;

	// Allocate page locked host memory for fast GPU transfer
	left_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC4);
	right_hm_ = cv::cuda::HostMem(height_, width_, CV_8UC4);
	hres_hm_ = cv::cuda::HostMem(fullheight_, fullwidth_, CV_8UC4);
}

PylonDevice::~PylonDevice() {

}

std::vector<ftl::rgbd::detail::DeviceDetails> PylonDevice::listDevices() {
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
}

bool PylonDevice::grab() {
	if (!isReady()) return false;

	//int dev;
	//cudaGetDevice(&dev);
	//LOG(INFO) << "Current cuda device = " << dev;

	try {
		FTL_Profile("Frame Capture", 0.001);
		if (rcam_) rcam_->WaitForFrameTriggerReady( 30, Pylon::TimeoutHandling_ThrowException);
		else lcam_->WaitForFrameTriggerReady( 30, Pylon::TimeoutHandling_ThrowException);

		lcam_->ExecuteSoftwareTrigger();
		if (rcam_) rcam_->ExecuteSoftwareTrigger();
	} catch (const GenericException &e) {
		LOG(ERROR) << "Pylon: Trigger exception - " << e.GetDescription();
		return false;
	}

	return true;
}

bool PylonDevice::get(cv::cuda::GpuMat &l_out, cv::cuda::GpuMat &r_out, cv::cuda::GpuMat &h_l, cv::Mat &h_r, Calibrate *c, cv::cuda::Stream &stream) {
	if (!isReady()) return false;

	Mat l, r ,hres;

	// Use page locked memory
	l = left_hm_.createMatHeader();
	r = right_hm_.createMatHeader();
	hres = hres_hm_.createMatHeader();

	Mat &lfull = (!hasHigherRes()) ? l : hres;
	Mat &rfull = (!hasHigherRes()) ? r : rtmp_;

	//ftl::cuda::setDevice();

	//int dev;
	//cudaGetDevice(&dev);
	//LOG(INFO) << "Current cuda device = " << dev;

	try {
		FTL_Profile("Frame Retrieve", 0.005);
		std::future<bool> future_b;
		if (rcam_) {
			future_b = std::move(ftl::pool.push([this,&rfull,&r,&l,c,&r_out,&h_r,&stream](int id) {
				Pylon::CGrabResultPtr result_right;
				int rcount = 0;
				if (rcam_ && rcam_->RetrieveResult(0, result_right, Pylon::TimeoutHandling_Return)) ++rcount;

				if (rcount == 0 || !result_right->GrabSucceeded()) {
					LOG(ERROR) << "Retrieve failed";
					return false;
				}

				cv::Mat wrap_right(
				result_right->GetHeight(),
				result_right->GetWidth(),
				CV_8UC1,
				(uint8_t*)result_right->GetBuffer());

				cv::cvtColor(wrap_right, rfull, cv::COLOR_BayerBG2BGRA);

				if (isStereo()) {
					c->rectifyRight(rfull);

					if (hasHigherRes()) {
						cv::resize(rfull, r, r.size(), 0.0, 0.0, cv::INTER_CUBIC);
						h_r = rfull;
					}
					else {
						h_r = Mat();
					}
				}

				r_out.upload(r, stream);
				return true;
			}));
		}

		Pylon::CGrabResultPtr result_left;
		int lcount = 0;
		{
			if (lcam_->RetrieveResult(0, result_left, Pylon::TimeoutHandling_Return)) ++lcount;
		}

		if (lcount == 0 || !result_left->GrabSucceeded()) {
			LOG(ERROR) << "Retrieve failed";
			return false;
		}

		cv::Mat wrap_left(
			result_left->GetHeight(),
			result_left->GetWidth(),
			CV_8UC1,
			(uint8_t*)result_left->GetBuffer());

		cv::cvtColor(wrap_left, lfull, cv::COLOR_BayerBG2BGRA);

		if (isStereo()) {
			c->rectifyLeft(lfull);
		}

		if (hasHigherRes()) {
			cv::resize(lfull, l, l.size(), 0.0, 0.0, cv::INTER_CUBIC);
			h_l.upload(hres, stream);
		} else {
			h_l = cv::cuda::GpuMat();
		}

		l_out.upload(l, stream);

		if (rcam_) {
			future_b.wait();
		}

	} catch (const GenericException &e) {
		LOG(ERROR) << "Pylon: An exception occurred - " << e.GetDescription();
	}

	return true;
}

bool PylonDevice::isReady() const {
    return lcam_ && lcam_->IsOpen();
}

