#include "camera.hpp"
#include "pose_window.hpp"
#include "screen.hpp"

#include <nanogui/glutil.h>

using ftl::rgbd::isValidDepth;
using ftl::gui::GLTexture;
using ftl::gui::PoseWindow;

// TODO(Nick) MOVE
class StatisticsImage {
private:
	cv::Mat data_;	// CV_32FC3, channels: m, s, f
	cv::Size size_;	// image size
	float n_;		// total number of samples

public:
	StatisticsImage(cv::Size size);
	StatisticsImage(cv::Size size, float max_f);

	/* @brief reset all statistics to 0
	 */
	void reset();

	/* @brief update statistics with new values
	 */
	void update(const cv::Mat &in);
	
	/* @brief variance (depth)
	 */
	void getVariance(cv::Mat &out);

	/* @brief standard deviation (depth)
	 */
	void getStdDev(cv::Mat &out);
	
	/* @brief mean value (depth)
	 */
	void getMean(cv::Mat &out);

	/* @brief percent of samples having valid depth value
	 */
	void getValidRatio(cv::Mat &out);
};

StatisticsImage::StatisticsImage(cv::Size size) :
	StatisticsImage(size, std::numeric_limits<float>::infinity()) {}

StatisticsImage::StatisticsImage(cv::Size size, float max_f) {
	size_ = size;
	n_ = 0.0f;
	data_ = cv::Mat(size, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));

	// TODO
	if (!std::isinf(max_f)) {
		LOG(WARNING) << "TODO: max_f_ not used. Values calculated for all samples";
	}
}

void StatisticsImage::reset() {
	n_ = 0.0f;
	data_ = cv::Scalar(0.0, 0.0, 0.0);
}

void StatisticsImage::update(const cv::Mat &in) {
	DCHECK(in.type() == CV_32F);
	DCHECK(in.size() == size_);
	
	n_ = n_ + 1.0f;

	// Welford's Method
	for (int row = 0; row < in.rows; row++) {
		float* ptr_data = data_.ptr<float>(row);
		const float* ptr_in = in.ptr<float>(row);

		for (int col = 0; col < in.cols; col++, ptr_in++) {
			float x = *ptr_in;
			float &m = *ptr_data++;
			float &s = *ptr_data++;
			float &f = *ptr_data++;
			float m_prev = m;

			if (!ftl::rgbd::isValidDepth(x)) continue;

			f = f + 1.0f;
			m = m + (x - m) / f;
			s = s + (x - m) * (x - m_prev);
		}
	}
}

void StatisticsImage::getVariance(cv::Mat &out) {
	std::vector<cv::Mat> channels(3);
	cv::split(data_, channels);
	cv::divide(channels[1], channels[2], out);
}

void StatisticsImage::getStdDev(cv::Mat &out) {
	getVariance(out);
	cv::sqrt(out, out);
}

void StatisticsImage::getMean(cv::Mat &out) {
	std::vector<cv::Mat> channels(3);
	cv::split(data_, channels);
	out = channels[0];
}

void StatisticsImage::getValidRatio(cv::Mat &out) {
	std::vector<cv::Mat> channels(3);
	cv::split(data_, channels);
	cv::divide(channels[2], n_, out);
}

static Eigen::Affine3d create_rotation_matrix(float ax, float ay, float az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * rx * ry;
}

ftl::gui::Camera::Camera(ftl::gui::Screen *screen, ftl::rgbd::Source *src) : screen_(screen), src_(src) {
	eye_ = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
	neye_ = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 0.0f);
	rotmat_.setIdentity();
	//up_ = Eigen::Vector3f(0,1.0f,0);
	lerpSpeed_ = 0.999f;
	sdepth_ = false;
	ftime_ = (float)glfwGetTime();
	pause_ = false;

	channel_ = ftl::rgbd::kChanLeft;

	channels_.push_back(ftl::rgbd::kChanLeft);
	channels_.push_back(ftl::rgbd::kChanDepth);

	// Create pose window...
	posewin_ = new PoseWindow(screen, src_->getURI());
	posewin_->setTheme(screen->windowtheme);
	posewin_->setVisible(false);

	src->setCallback([this](int64_t ts, cv::Mat &rgb, cv::Mat &depth) {
		UNIQUE_LOCK(mutex_, lk);
		rgb_.create(rgb.size(), rgb.type());
		depth_.create(depth.size(), depth.type());
		cv::swap(rgb_,rgb);
		cv::swap(depth_, depth);
	});
}

ftl::gui::Camera::~Camera() {

}

ftl::rgbd::Source *ftl::gui::Camera::source() {
	return src_;
}

void ftl::gui::Camera::setPose(const Eigen::Matrix4d &p) {
	eye_[0] = p(0,3);
	eye_[1] = p(1,3);
	eye_[2] = p(2,3);

	double sx = Eigen::Vector3d(p(0,0), p(1,0), p(2,0)).norm();
	double sy = Eigen::Vector3d(p(0,1), p(1,1), p(2,1)).norm();
	double sz = Eigen::Vector3d(p(0,2), p(1,2), p(2,2)).norm();

	Eigen::Matrix4d rot = p;
	rot(0,3) = 0.0;
	rot(1,3) = 0.0;
	rot(2,3) = 0.0;
	rot(0,0) = rot(0,0) / sx;
	rot(1,0) = rot(1,0) / sx;
	rot(2,0) = rot(2,0) / sx;
	rot(0,1) = rot(0,1) / sy;
	rot(1,1) = rot(1,1) / sy;
	rot(2,1) = rot(2,1) / sy;
	rot(0,2) = rot(0,2) / sz;
	rot(1,2) = rot(1,2) / sz;
	rot(2,2) = rot(2,2) / sz;
	rotmat_ = rot;
}

void ftl::gui::Camera::mouseMovement(int rx, int ry, int button) {
	if (!src_->hasCapabilities(ftl::rgbd::kCapMovable)) return;
	if (button == 1) {
		float rrx = ((float)ry * 0.2f * delta_);
		//orientation_[2] += std::cos(orientation_[1])*((float)rel[1] * 0.2f * delta_);
		float rry = (float)rx * 0.2f * delta_;
		float rrz = 0.0;


		Eigen::Affine3d r = create_rotation_matrix(rrx, -rry, rrz);
		rotmat_ = rotmat_ * r.matrix();
	}
}

void ftl::gui::Camera::keyMovement(int key, int modifiers) {
	if (!src_->hasCapabilities(ftl::rgbd::kCapMovable)) return;
	if (key == 263 || key == 262) {
		float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
		float scalar = (key == 263) ? -mag : mag;
		neye_ += rotmat_*Eigen::Vector4d(scalar,0.0,0.0,1.0);
		return;
	} else if (key == 264 || key == 265) {
		float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
		float scalar = (key == 264) ? -mag : mag;
		neye_ += rotmat_*Eigen::Vector4d(0.0,0.0,scalar,1.0);
		return;
	} else if (key == 266 || key == 267) {
		float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
		float scalar = (key == 266) ? -mag : mag;
		neye_ += rotmat_*Eigen::Vector4d(0.0,scalar,0.0,1.0);
		return;
	}
}

void ftl::gui::Camera::showPoseWindow() {
	posewin_->setVisible(true);
}

void ftl::gui::Camera::showSettings() {

}

void ftl::gui::Camera::setChannel(ftl::rgbd::channel_t c) {
	channel_ = c;
	switch (c) {
	case ftl::rgbd::kChanEnergy:
	case ftl::rgbd::kChanFlow:
	case ftl::rgbd::kChanConfidence:
	case ftl::rgbd::kChanNormals:
	case ftl::rgbd::kChanRight:
		src_->setChannel(c);
		break;

	case ftl::rgbd::kChanDeviation:
		if (stats_) { stats_->reset(); }
		src_->setChannel(ftl::rgbd::kChanDepth);
		break;
	
	case ftl::rgbd::kChanDepth:
		src_->setChannel(c);
		break;
	
	default: src_->setChannel(ftl::rgbd::kChanNone);
	}
}

static void visualizeDepthMap(	const cv::Mat &depth, cv::Mat &out,
								const float max_depth)
{
	DCHECK(max_depth > 0.0);

	depth.convertTo(out, CV_8U, 255.0f / max_depth);
	out = 255 - out;
	cv::Mat mask = (depth >= 39.0f); // TODO (mask for invalid pixels)
	
	applyColorMap(out, out, cv::COLORMAP_JET);
	out.setTo(cv::Scalar(255, 255, 255), mask);
}

static void visualizeEnergy(	const cv::Mat &depth, cv::Mat &out,
								const float max_depth)
{
	DCHECK(max_depth > 0.0);

	depth.convertTo(out, CV_8U, 255.0f / max_depth);
	//out = 255 - out;
	cv::Mat mask = (depth >= 39.0f); // TODO (mask for invalid pixels)
	
	applyColorMap(out, out, cv::COLORMAP_JET);
	out.setTo(cv::Scalar(255, 255, 255), mask);
}

static void drawEdges(	const cv::Mat &in, cv::Mat &out,
						const int ksize = 3, double weight = -1.0, const int threshold = 32,
						const int threshold_type = cv::THRESH_TOZERO)
{
	cv::Mat edges;
	cv::Laplacian(in, edges, 8, ksize);
	cv::threshold(edges, edges, threshold, 255, threshold_type);

	cv::Mat edges_color(in.size(), CV_8UC3);
	cv::addWeighted(edges, weight, out, 1.0, 0.0, out, CV_8UC3);
}

bool ftl::gui::Camera::thumbnail(cv::Mat &thumb) {
	UNIQUE_LOCK(mutex_, lk);
	src_->grab(1,9);
	if (rgb_.empty()) return false;
	cv::resize(rgb_, thumb, cv::Size(320,180));
	return true;
}

const GLTexture &ftl::gui::Camera::captureFrame() {
	float now = (float)glfwGetTime();
	delta_ = now - ftime_;
	ftime_ = now;

	if (src_ && src_->isReady()) {
		UNIQUE_LOCK(mutex_, lk);

		// Lerp the Eye
		eye_[0] += (neye_[0] - eye_[0]) * lerpSpeed_ * delta_;
		eye_[1] += (neye_[1] - eye_[1]) * lerpSpeed_ * delta_;
		eye_[2] += (neye_[2] - eye_[2]) * lerpSpeed_ * delta_;

		Eigen::Translation3d trans(eye_);
		Eigen::Affine3d t(trans);
		Eigen::Matrix4d viewPose = t.matrix() * rotmat_;

		if (src_->hasCapabilities(ftl::rgbd::kCapMovable)) src_->setPose(viewPose);
		src_->grab();
		//src_->getFrames(rgb, depth);

		// When switching from right to depth, client may still receive
		// right images from previous batch (depth.channels() == 1 check)
		if (channel_ == ftl::rgbd::kChanDeviation &&
			depth_.rows > 0 && depth_.channels() == 1)
		{
			if (!stats_) {
				stats_ = new StatisticsImage(depth_.size());
			}
			
			stats_->update(depth_);
		}

		cv::Mat tmp;

		switch(channel_) {
			case ftl::rgbd::kChanEnergy:
				if (depth_.rows == 0) { break; }
				visualizeEnergy(depth_, tmp, 10.0);
				texture_.update(tmp);
				break;
			case ftl::rgbd::kChanDepth:
				if (depth_.rows == 0) { break; }
				visualizeDepthMap(depth_, tmp, 7.0);
				if (screen_->root()->value("showEdgesInDepth", false)) drawEdges(rgb_, tmp);
				texture_.update(tmp);
				break;
			
			case ftl::rgbd::kChanDeviation:
				if (depth_.rows == 0) { break; }
				//imageSize = Vector2f(depth.cols, depth.rows);
				stats_->getStdDev(tmp);
				tmp.convertTo(tmp, CV_8U, 1000.0);
				applyColorMap(tmp, tmp, cv::COLORMAP_HOT);
				texture_.update(tmp);
				break;

		case ftl::rgbd::kChanFlow:
		case ftl::rgbd::kChanConfidence:
		case ftl::rgbd::kChanNormals:
			case ftl::rgbd::kChanRight:
				if (depth_.rows == 0 || depth_.type() != CV_8UC3) { break; }
				texture_.update(depth_);
				break;

			default:
				if (rgb_.rows == 0) { break; }
				//imageSize = Vector2f(rgb.cols,rgb.rows);
				texture_.update(rgb_);
		}
	}

	return texture_;
}

nlohmann::json ftl::gui::Camera::getMetaData() {
	return nlohmann::json();
}
