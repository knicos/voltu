#include "middlebury_source.hpp"

#include "disparity.hpp"

using ftl::rgbd::detail::MiddleburySource;
using ftl::rgbd::detail::Disparity;
using std::string;

MiddleburySource::MiddleburySource(ftl::rgbd::Source *host)
		: ftl::rgbd::detail::Source(host), ready_(false) {
	// Not VALID
}

static bool loadMiddleburyCalib(const std::string &filename, ftl::rgbd::Camera &params, double scaling) {
	FILE* fp = fopen(filename.c_str(), "r");
	char buff[512];
	
	float cam0[3][3];
	float cam1[3][3];
	float doffs;
	float baseline;
	int width;
	int height;
	int ndisp;
	int isint;
	int vmin;
	int vmax;
	float dyavg;
	float dymax;

	if (fp != nullptr)
	{
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "cam0 = [%f %f %f; %f %f %f; %f %f %f]\n", &cam0[0][0], &cam0[0][1], &cam0[0][2], &cam0[1][0], &cam0[1][1], &cam0[1][2], &cam0[2][0], &cam0[2][1], &cam0[2][2]);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "cam1 = [%f %f %f; %f %f %f; %f %f %f]\n", &cam1[0][0], &cam1[0][1], &cam1[0][2], &cam1[1][0], &cam1[1][1], &cam1[1][2], &cam1[2][0], &cam1[2][1], &cam1[2][2]);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "doffs = %f\n", &doffs);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "baseline = %f\n", &baseline);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "width = %d\n", &width);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "height = %d\n", &height);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "ndisp = %d\n", &ndisp);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "isint = %d\n", &isint);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "vmin = %d\n", &vmin);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "vmax = %d\n", &vmax);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "dyavg = %f\n", &dyavg);
		if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "dymax = %f\n", &dymax);
		fclose(fp);

		params.fx = cam0[0][0] * scaling;
		params.fy = params.fx;
		params.cx = -cam0[0][2] * scaling;
		params.cy = -cam0[1][2] * scaling;
		params.width = width * scaling;
		params.height = height * scaling;
		params.baseline = baseline;

		return true;
	}

	return false;
}

MiddleburySource::MiddleburySource(ftl::rgbd::Source *host, const string &dir)
		: ftl::rgbd::detail::Source(host), ready_(false) {

	double scaling = host->value("scaling", 0.5);

	capabilities_ = kCapStereo;

	// Load params from txt file..
	/*params_.fx = 3000.0 * scaling;
	params_.width = 3000.0 * scaling;
	params_.height = 1920.0 * scaling;
	params_.baseline = 237.0; // * scaling;
	params_.fy = params_.fx;
	params_.cx = -1146.717 * scaling;
	params_.cy = -975.476 * scaling;*/

	if (!loadMiddleburyCalib(dir+"/calib.txt", params_, scaling)) {
		LOG(ERROR) << "Could not load middlebury calibration";
		return;
	}


	// Add calibration to config object
	host_->getConfig()["focal"] = params_.fx;
	host_->getConfig()["centre_x"] = params_.cx;
	host_->getConfig()["centre_y"] = params_.cy;
	host_->getConfig()["baseline"] = params_.baseline;

	// Add event handlers to allow calibration changes...
	host_->on("baseline", [this](const ftl::config::Event &e) {
		params_.baseline = host_->value("baseline", params_.baseline);
	});

	host_->on("focal", [this](const ftl::config::Event &e) {
		params_.fx = host_->value("focal", params_.fx);
		params_.fy = params_.fx;
	});

	// left and right masks (areas outside rectified images)
	// only left mask used
	cv::cuda::GpuMat mask_r_gpu(params_.height, params_.width, CV_8U, 255);
	cv::cuda::GpuMat mask_l_gpu(params_.height, params_.width, CV_8U, 255);
	
	//calib_->rectifyStereo(mask_l_gpu, mask_r_gpu, stream_);
	//stream_.waitForCompletion();

	cv::Mat mask_l;
	mask_l_gpu.download(mask_l);
	mask_l_ = (mask_l == 0);

	if (!host_->getConfig()["disparity"].is_object()) {
		host_->getConfig()["disparity"] = {{"algorithm","libsgm"}};
	}
	
	disp_ = Disparity::create(host_, "disparity");
    if (!disp_) LOG(FATAL) << "Unknown disparity algorithm : " << *host_->get<ftl::config::json_t>("disparity");
	disp_->setMask(mask_l_);

	// Load image files...
	cv::Mat right_tmp;
	rgb_ = cv::imread(dir+"/im0.png", cv::IMREAD_COLOR);
	right_tmp = cv::imread(dir+"/im1.png", cv::IMREAD_COLOR);

	cv::resize(rgb_, rgb_, cv::Size(params_.width, params_.height));
	cv::resize(right_tmp, right_tmp, cv::Size(params_.width, params_.height));

	left_.upload(rgb_);
	right_.upload(right_tmp);

	_performDisparity();
	ready_ = true;
}

static void disparityToDepth(const cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &depth,
							 const ftl::rgbd::Camera &c, cv::cuda::Stream &stream) {
	double val = c.baseline * c.fx;
	cv::cuda::divide(val, disparity, depth, 1.0f / 1000.0f, -1, stream);
}

void MiddleburySource::_performDisparity() {
	if (depth_tmp_.empty()) depth_tmp_ = cv::cuda::GpuMat(left_.size(), CV_32FC1);
	if (disp_tmp_.empty()) disp_tmp_ = cv::cuda::GpuMat(left_.size(), CV_32FC1);
	//calib_->rectifyStereo(left_, right_, stream_);
	disp_->compute(left_, right_, disp_tmp_, stream_);
	disparityToDepth(disp_tmp_, depth_tmp_, params_, stream_);
	//left_.download(rgb_, stream_);
	//rgb_ = lsrc_->cachedLeft();
	depth_tmp_.download(depth_, stream_);

	stream_.waitForCompletion();
}

bool MiddleburySource::grab(int n, int b) {
	//_performDisparity();
	return true;
}

