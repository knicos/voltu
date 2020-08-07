#include <loguru.hpp>

#include <unordered_set>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include <ftl/configuration.hpp>
#include <ftl/profiler.hpp>

#include <nlohmann/json.hpp>

#ifdef HAVE_OPTFLOW
#include <ftl/operators/opticalflow.hpp>
#endif

#include <ftl/operators/smoothing.hpp>
#include <ftl/operators/colours.hpp>
#include <ftl/operators/normals.hpp>
#include <ftl/operators/filling.hpp>
#include <ftl/operators/segmentation.hpp>
#include <ftl/operators/disparity.hpp>
#include <ftl/operators/mask.hpp>

#include <ftl/rgbd/capabilities.hpp>
#include <ftl/codecs/shapes.hpp>
#include <ftl/calibration/structures.hpp>
#include <ftl/calibration/parameters.hpp>

#include "stereovideo.hpp"
#include "ftl/threads.hpp"
#include "rectification.hpp"

#include "opencv.hpp"

#ifdef HAVE_PYLON
#include "pylon.hpp"
#endif

using ftl::rgbd::detail::StereoVideoSource;
using ftl::codecs::Channel;
using std::string;
using ftl::rgbd::Capability;


static cv::Mat rmat(const cv::Vec3d &rvec) {
	cv::Mat R(cv::Size(3, 3), CV_64FC1);
	cv::Rodrigues(rvec, R);
	return R;
}

static Eigen::Matrix4d matrix(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
	cv::Mat M = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
	rmat(rvec).copyTo(M(cv::Rect(0, 0, 3, 3)));
	M.at<double>(0, 3) = tvec[0];
	M.at<double>(1, 3) = tvec[1];
	M.at<double>(2, 3) = tvec[2];
	Eigen::Matrix4d r;
	cv::cv2eigen(M,r);
	return r;
}


ftl::rgbd::detail::Device::Device(nlohmann::json &config) : Configurable(config) {

}

ftl::rgbd::detail::Device::~Device() {

}

StereoVideoSource::StereoVideoSource(ftl::rgbd::Source *host)
		: ftl::rgbd::BaseSourceImpl(host), ready_(false) {
	auto uri = host->get<std::string>("uri");
	if (uri) {
		init(*uri);
	} else {
		init("");
	}

}

StereoVideoSource::StereoVideoSource(ftl::rgbd::Source *host, const string &file)
		: ftl::rgbd::BaseSourceImpl(host), ready_(false) {

	init(file);
}

StereoVideoSource::~StereoVideoSource() {
	delete lsrc_;
	if (pipeline_input_) delete pipeline_input_;
}

bool StereoVideoSource::supported(const std::string &dev) {
	if (dev == "pylon") {
		#ifdef HAVE_PYLON
		auto pylon_devices = ftl::rgbd::detail::PylonDevice::listDevices();
		return pylon_devices.size() > 0;
		#else
		return false;
		#endif
	} else if (dev == "video") {
		return true;
	} else if (dev == "camera") {
		return ftl::rgbd::detail::OpenCVDevice::getDevices().size() > 0;
	} else if (dev == "stereo") {
		return ftl::rgbd::detail::OpenCVDevice::getDevices().size() > 1;
	}

	return false;
}

void StereoVideoSource::init(const string &file) {
	capabilities_ = kCapVideo | kCapStereo;

	ftl::URI uri(file);

	if (uri.getScheme() == ftl::URI::SCHEME_DEVICE) {
		if (uri.getPathSegment(0) == "pylon") {
			#ifdef HAVE_PYLON
			LOG(INFO) << "Using Pylon...";
			lsrc_ = ftl::create<ftl::rgbd::detail::PylonDevice>(host_, "feed");
			#else
			throw FTL_Error("Not built with pylon support");
			#endif
		} else if (uri.getPathSegment(0) == "opencv") {
			// Use cameras
			LOG(INFO) << "Using OpenCV cameras...";
			lsrc_ = ftl::create<ftl::rgbd::detail::OpenCVDevice>(host_, "feed", true);
		} else if (uri.getPathSegment(0) == "video" || uri.getPathSegment(0) == "camera") {
			// Use cameras
			LOG(INFO) << "Using OpenCV camera...";
			lsrc_ = ftl::create<ftl::rgbd::detail::OpenCVDevice>(host_, "feed", false);
		} else if (uri.getPathSegment(0) == "stereo") {
			// Use cameras
			LOG(INFO) << "Using OpenCV cameras...";
			lsrc_ = ftl::create<ftl::rgbd::detail::OpenCVDevice>(host_, "feed", true);
		}
	}

	if (!lsrc_) return;  // throw?

	color_size_ = cv::Size(lsrc_->width(), lsrc_->height());

	pipeline_input_ = ftl::config::create<ftl::operators::Graph>(host_, "input");
	#ifdef HAVE_OPTFLOW
	pipeline_input_->append<ftl::operators::NVOpticalFlow>("optflow");
	#endif
	pipeline_input_->append<ftl::operators::ColourChannels>("colour");

	cv::Size size_full = cv::Size(lsrc_->fullWidth(), lsrc_->fullHeight());
	rectification_ = std::unique_ptr<StereoRectification>
		(ftl::create<StereoRectification>(host_, "rectification", size_full));

	string fname_default = "calibration.yml";
	auto fname_config = host_->get<string>("calibration");
	string fname = fname_config ? *fname_config : fname_default;
	auto calibf = ftl::locateFile(fname);
	if (calibf) {
		fname_calib_ = *calibf;
		calibration_ = ftl::calibration::CalibrationData::readFile(fname_calib_);
		calibration_.enabled = host_->value("rectify", calibration_.enabled);
		rectification_->setCalibration(calibration_);
		rectification_->setEnabled(calibration_.enabled);
	}
	else {
		fname_calib_ = fname_config ?	*fname_config :
										string(FTL_LOCAL_CONFIG_ROOT) + "/"
										+ std::string("calibration.yml");

		LOG(ERROR) << "No calibration file found, calibration will be saved to " + fname;
	}

	// Generate camera parameters for next frame
	do_update_params_ = true;

	LOG(INFO) << "StereoVideo source ready...";
	ready_ = true;

	host_->on("size", [this]() {
		do_update_params_ = true;
	});

	host_->on("rectify", [this]() {
		calibration_.enabled = host_->value("rectify", true);
		rectification_->setEnabled(calibration_.enabled);
		do_update_params_ = true;
	});

	rectification_->setInterpolation(
		host_->value("rectify_inter_cubic", false) ? cv::INTER_CUBIC : cv::INTER_LINEAR);

	host_->on("rectify_inter_cubic", [this]() {
		bool v = host_->value("rectify_inter_cubic", false);
		rectification_->setInterpolation(v ? cv::INTER_CUBIC : cv::INTER_LINEAR);
	});

	host_->on("offset_z", [this]() {
		do_update_params_ = true;
	});
}

void StereoVideoSource::updateParameters(ftl::rgbd::Frame &frame) {
	auto &meta = frame.create<std::map<std::string,std::string>>(Channel::MetaData);
	meta["name"] = host_->value("name", host_->getID());
	meta["id"] = host_->getID();
	meta["uri"] = host_->value("uri", std::string(""));

	if (lsrc_) lsrc_->populateMeta(meta);

	if (!frame.has(Channel::Capabilities)) {
		auto &cap = frame.create<std::unordered_set<Capability>>(Channel::Capabilities);
		cap.emplace(Capability::VIDEO);
		cap.emplace(Capability::LIVE);
	}

	frame.create<ftl::calibration::CalibrationData>(Channel::CalibrationData) = calibration_;

	calibration_change_ = frame.onChange(Channel::CalibrationData, [this]
			(ftl::data::Frame& frame, ftl::codecs::Channel) {

		if (!lsrc_->isStereo()) return true;

		auto &change = frame.get<ftl::calibration::CalibrationData>(Channel::CalibrationData);
		try {
			change.writeFile(fname_calib_);
		}
		catch (...) {
			LOG(ERROR) << "Saving calibration to file failed";
		}

		calibration_ = ftl::calibration::CalibrationData(change);
		rectification_->setCalibration(calibration_);
		rectification_->setEnabled(change.enabled);

		do_update_params_ = true;
		return true;
	});

	if (lsrc_->isStereo()) {
		Eigen::Matrix4d pose;
		// NOTE: pose update (new origin/rotation)
		cv::cv2eigen(calibration_.origin * rectification_->getPose(Channel::Left), pose);
		frame.setPose() = pose;

		cv::Mat K = rectification_->cameraMatrix(color_size_);
		float fx = static_cast<float>(K.at<double>(0,0));

		float baseline = static_cast<float>(rectification_->baseline());
		float doff = rectification_->doff(color_size_);

		double d_resolution = this->host_->getConfig().value<double>("depth_resolution", 0.0);
		float min_depth = this->host_->getConfig().value<double>("min_depth", 0.45);
		float max_depth = this->host_->getConfig().value<double>("max_depth", (lsrc_->isStereo()) ? 12.0 : 1.0);

		if (d_resolution > 0.0) {
			// Learning OpenCV p. 442; TODO: remove. should not be applied here
			float max_depth_new = sqrt(d_resolution * fx * baseline);
			max_depth = (max_depth_new > max_depth) ? max_depth : max_depth_new;
		}

		auto& params = frame.setLeft();
		params = {
			fx,
			static_cast<float>(K.at<double>(1,1)),	// Fy
			static_cast<float>(-K.at<double>(0,2)),	// Cx
			static_cast<float>(-K.at<double>(1,2)),	// Cy
			(unsigned int) color_size_.width,
			(unsigned int) color_size_.height,
			min_depth,
			max_depth,
			baseline,
			doff
		};

		host_->getConfig()["focal"] = params.fx;
		host_->getConfig()["centre_x"] = params.cx;
		host_->getConfig()["centre_y"] = params.cy;
		host_->getConfig()["baseline"] = params.baseline;
		host_->getConfig()["doffs"] = params.doffs;

	} else {
		Eigen::Matrix4d pose;
		auto& params = frame.setLeft();

		params.cx = -(color_size_.width / 2.0);
		params.cy = -(color_size_.height / 2.0);
		params.fx = 700.0;
		params.fy = 700.0;
		params.maxDepth = host_->value("size", 1.0f);
		params.minDepth = 0.0f;
		params.doffs = 0.0;
		params.baseline = 0.1f;
		params.width = color_size_.width;
		params.height = color_size_.height;;

		float offsetz = host_->value("offset_z", 0.0f);
		//state_.setPose(matrix(cv::Vec3d(0.0, 3.14159, 0.0), cv::Vec3d(0.0,0.0,params_.maxDepth+offsetz)));
		pose = matrix(cv::Vec3d(0.0, 3.14159, 0.0), cv::Vec3d(0.0,0.0,params.maxDepth + offsetz));

		/*host_->on("size", [this](const ftl::config::Event &e) {
			float offsetz = host_->value("offset_z",0.0f);
			params_.maxDepth = host_->value("size", 1.0f);
			//state_.getLeft() = params_;
			pose_ = matrix(cv::Vec3d(0.0, 3.14159, 0.0), cv::Vec3d(0.0,0.0,params_.maxDepth+offsetz));
			do_update_params_ = true;
		});*/

		frame.setPose() = pose;
	}

	const auto& params = frame.getLeft();
	CHECK(params.fx > 0) << "focal length must be positive";
	CHECK(params.fy > 0) << "focal length must be positive";
	CHECK(params.cx < 0) << "bad principal point coordinate (negative value)";
	CHECK(-params.cx < params.width) << "principal point must be inside image";
	CHECK(params.cy < 0) << "bad principal point coordinate (negative value)";
	CHECK(-params.cy < params.height) << "principal point must be inside image";
	CHECK(params.baseline >= 0.0) << "baseline must be positive";
}

bool StereoVideoSource::capture(int64_t ts) {
	cap_status_ = lsrc_->grab();
	return cap_status_;
}

bool StereoVideoSource::retrieve(ftl::rgbd::Frame &frame) {
	FTL_Profile("Stereo Retrieve", 0.03);

	if (!cap_status_) return false;

	if (host_->value("add_right_pose", false)) {
		auto shapes = frame.create<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);
		Eigen::Matrix4d pose;
		cv::cv2eigen(rectification_->getPose(Channel::Right), pose);
		Eigen::Matrix4f posef = pose.cast<float>();
		shapes.list.push_back(ftl::codecs::Shape3D{
				1,
				ftl::codecs::Shape3DType::CAMERA,
				Eigen::Vector3f{0.2, 0.2, 0.2},
				posef,
				std::string("Right Camera")
		});
	}

	if (do_update_params_) {
		updateParameters(frame);
		do_update_params_ = false;
	}

	cv::cuda::GpuMat gpu_dummy;
	cv::Mat dummy;
	auto &hres = (lsrc_->hasHigherRes()) ? frame.create<cv::cuda::GpuMat>(Channel::ColourHighRes) : gpu_dummy;
	auto &hres_r = (lsrc_->hasHigherRes()) ? frame.create<cv::Mat>(Channel::RightHighRes) : dummy;

	if (lsrc_->isStereo()) {
		cv::cuda::GpuMat &left = frame.create<cv::cuda::GpuMat>(Channel::Left);
		cv::cuda::GpuMat &right = frame.create<cv::cuda::GpuMat>(Channel::Right);
		if (!lsrc_->get(frame, left, right, hres, hres_r, rectification_.get(), stream2_)) {
			frame.remove(Channel::Left);
			frame.remove(Channel::Right);
		}
	}
	else {
		cv::cuda::GpuMat &left = frame.create<cv::cuda::GpuMat>(Channel::Left);
		cv::cuda::GpuMat right;
		if (!lsrc_->get(frame, left, right, hres, hres_r, rectification_.get(), stream2_)) {
			frame.remove(Channel::Left);
		}
	}

	//LOG(INFO) << "Channel size: " << hres.size();

	pipeline_input_->apply(frame, frame, cv::cuda::StreamAccessor::getStream(stream2_));
	stream2_.waitForCompletion();

	return true;
}

bool StereoVideoSource::isReady() {
	return ready_;
}
