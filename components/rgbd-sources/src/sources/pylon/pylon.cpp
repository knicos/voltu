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
        : ftl::rgbd::detail::Source(host), ready_(false), lcam_(nullptr), rcam_(nullptr) {
	capabilities_ = kCapVideo;

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

void PylonSource::_configureCamera(CBaslerUniversalInstantCamera *cam) {
	// Get the camera control object.
	GenApi::INodeMap& nodemap = cam->GetNodeMap();
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
}

bool PylonSource::capture(int64_t ts) {
	timestamp_ = ts;
	if (!isReady()) return false;

	try {
		lcam_->WaitForFrameTriggerReady( 30, Pylon::TimeoutHandling_ThrowException);
		if (rcam_) rcam_->WaitForFrameTriggerReady( 30, Pylon::TimeoutHandling_ThrowException);

		lcam_->ExecuteSoftwareTrigger();
		if (rcam_) rcam_->ExecuteSoftwareTrigger();
	} catch (const GenericException &e) {
		LOG(ERROR) << "Pylon: Trigger exception - " << e.GetDescription();
	}

	return true;
}

bool PylonSource::retrieve() {
	if (!isReady()) return false;

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

		Pylon::CGrabResultPtr result_left;
		Pylon::CGrabResultPtr result_right;

		int lcount = 0;
		if (lcam_->RetrieveResult(0, result_left, Pylon::TimeoutHandling_Return)) ++lcount;

		int rcount = 0;
		if (rcam_ && rcam_->RetrieveResult(0, result_right, Pylon::TimeoutHandling_Return)) ++rcount;

		if (lcount == 0 || !result_left->GrabSucceeded()) {
			LOG(ERROR) << "Retrieve failed";
			return false;
		}

		cv::Mat wrap_left(
			result_left->GetHeight(),
			result_left->GetWidth(),
			CV_8UC1,
			(uint8_t*)result_left->GetBuffer());

		cv::cvtColor(wrap_left, tmp_, cv::COLOR_BayerBG2BGRA);
		frame.create<cv::cuda::GpuMat>(ftl::codecs::Channel::Colour).upload(tmp_);

		if (rcount > 0 && result_right->GrabSucceeded()) {
			cv::Mat wrap_right(
			result_right->GetHeight(),
			result_right->GetWidth(),
			CV_8UC1,
			(uint8_t*)result_right->GetBuffer());

			cv::cvtColor(wrap_right, tmp_, cv::COLOR_BayerBG2BGRA);
			frame.create<cv::cuda::GpuMat>(ftl::codecs::Channel::Colour2).upload(tmp_);
		}

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

bool PylonSource::compute(int64_t ts) {
	auto &frame = frames_[1];
	host_->notify(ts, frame);
    return true;
}

bool PylonSource::isReady() {
    return lcam_ && lcam_->IsOpen();
}

