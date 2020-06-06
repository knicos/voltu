#include "pylon.hpp"

#include <loguru.hpp>
#include <ftl/threads.hpp>
#include <ftl/rgbd/source.hpp>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include <opencv2/imgproc.hpp>

using ftl::rgbd::detail::PylonSource;
using std::string;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;
using namespace Pylon;

PylonSource::PylonSource(ftl::rgbd::Source *host)
        : ftl::rgbd::detail::Source(host), ready_(false), lcam_(nullptr) {
	capabilities_ = kCapVideo;

	auto &inst = CTlFactory::GetInstance();

	Pylon::DeviceInfoList_t devices;
	inst.EnumerateDevices(devices);

	if (devices.size() == 0) {
		LOG(ERROR) << "No Pylon devices attached";
		return;
	} else {
		for (auto d : devices) {
			LOG(INFO) << " - found Pylon device - " << d.GetModelName();
		}
	}

	try {
    	lcam_ = new CBaslerUniversalInstantCamera( CTlFactory::GetInstance().CreateFirstDevice());

		lcam_->RegisterConfiguration( new Pylon::CSoftwareTriggerConfiguration, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);

		lcam_->Open();

		// Get the camera control object.
		GenApi::INodeMap& nodemap = lcam_->GetNodeMap();
		// Get the parameters for setting the image area of interest (Image AOI).
		CIntegerParameter width(nodemap, "Width");
		CIntegerParameter height(nodemap, "Height");
		CIntegerParameter offsetX(nodemap, "OffsetX");
		CIntegerParameter offsetY(nodemap, "OffsetY");

		params_.width = width.GetValue();
		params_.height = height.GetValue();

		LOG(INFO) << "Camera resolution = " << params_.width << "x" << params_.height;

		// Set the pixel data format.
		CEnumParameter format(nodemap, "PixelFormat");
		LOG(INFO) << "Camera format: " << format.GetValue();

		if (format.CanSetValue("BayerBG8")) {  // YCbCr422_8
			format.SetValue("BayerBG8");
		} else {
			LOG(WARNING) << "Could not change pixel format";
		}

		lcam_->StartGrabbing( Pylon::GrabStrategy_OneByOne);

		ready_ = true;
	} catch (const Pylon::GenericException &e) {
		// Error handling.
        LOG(ERROR) << "Pylon: An exception occurred - " << e.GetDescription();
	}

    /*params_.width = intrin.width;
    params_.height = intrin.height;
    params_.cx = -intrin.ppx;
    params_.cy = -intrin.ppy;
    params_.fx = intrin.fx;
    params_.fy = intrin.fy;
    params_.maxDepth = 3.0;
    params_.minDepth = 0.1;
	params_.doffs = 0.0;*/

    state_.getLeft() = params_;
}

PylonSource::~PylonSource() {

}

bool PylonSource::capture(int64_t ts) {
	timestamp_ = ts;
	if (!lcam_) return false;

	try {
		if ( lcam_->WaitForFrameTriggerReady( 50, Pylon::TimeoutHandling_ThrowException)) {
			lcam_->ExecuteSoftwareTrigger();
			//LOG(INFO) << "TRIGGER";
		}
	} catch (const GenericException &e) {
		LOG(ERROR) << "Pylon: Trigger exception - " << e.GetDescription();
	}

	return true;
}

bool PylonSource::retrieve() {
	if (!lcam_) return false;

	auto &frame = frames_[0];
	frame.reset();
	frame.setOrigin(&state_);

	try {
		/*if ( lcam_->GetGrabResultWaitObject().Wait(40)) {
			LOG(INFO) << "Grad result waiting";
		} else {
			LOG(INFO) << "No results";
			return false;
		}*/

		Pylon::CGrabResultPtr ptrGrabResult;

		int count = 0;
		if (lcam_->RetrieveResult(0, ptrGrabResult, Pylon::TimeoutHandling_Return)) ++count;

		if (count == 0 || !ptrGrabResult->GrabSucceeded()) {
			LOG(ERROR) << "Retrieve failed";
			return false;
		}

		cv::Mat wrap(
			ptrGrabResult->GetHeight(),
			ptrGrabResult->GetWidth(),
			CV_8UC1,
			(uint8_t*)ptrGrabResult->GetBuffer());

		cv::cvtColor(wrap, tmp_, cv::COLOR_BayerBG2BGRA);
		frame.create<cv::cuda::GpuMat>(ftl::codecs::Channel::Colour).upload(tmp_);

	} catch (const GenericException &e) {
		LOG(ERROR) << "Pylon: An exception occurred - " << e.GetDescription();
	}

	return true;
}

void PylonSource::swap() {
	auto tmp = std::move(frames_[0]);
	frames_[0] = std::move(frames_[1]);
	frames_[1] = std::move(tmp);
}

bool PylonSource::compute(int n, int b) {
	auto &frame = frames_[1];
	host_->notify(timestamp_, frame);
    return true;
}

bool PylonSource::isReady() {
    return true;
}

