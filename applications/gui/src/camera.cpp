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
	cv::Mat data_;
	cv::Size size_;
	float max_f_;

public:
	StatisticsImage(cv::Size size);
	StatisticsImage(cv::Size size, float max_f);

	void reset();
	void update(const cv::Mat &in);
	void getVariance(cv::Mat &out);
	void getStdDev(cv::Mat &out);
	void getMean(cv::Mat &out);
};

StatisticsImage::StatisticsImage(cv::Size size) :
	StatisticsImage(size, std::numeric_limits<float>::infinity()) {}

StatisticsImage::StatisticsImage(cv::Size size, float max_f) {
	size_ = size;
	// channels: m, s, f
	data_ = cv::Mat(size, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
	max_f_ = max_f;
	// TODO
	if (!std::isinf(max_f_)) {
		LOG(WARNING) << "TODO: max_f_ not used. Values calculated for all samples";
	}
}

void StatisticsImage::reset() {
	data_ = cv::Scalar(0.0, 0.0, 0.0);
}

void StatisticsImage::update(const cv::Mat &in) {
	DCHECK(in.type() == CV_32F);
	DCHECK(in.size() == size_);
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
	depth_ = false;
	ftime_ = (float)glfwGetTime();
	pause_ = false;

	channel_ = ftl::rgbd::kChanLeft;

	channels_.push_back(ftl::rgbd::kChanLeft);
	channels_.push_back(ftl::rgbd::kChanDepth);

	// Create pose window...
	posewin_ = new PoseWindow(screen, src_->getURI());
	posewin_->setTheme(screen->windowtheme);
	posewin_->setVisible(false);
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
	case ftl::rgbd::kChanRight:
		[[fallthrough]];
	
	case ftl::rgbd::kChanDeviation:
		if (stats_) { stats_->reset(); }
		[[fallthrough]];
	
	case ftl::rgbd::kChanDepth:
		src_->setChannel(ftl::rgbd::kChanDepth);
		break;
	
	default: src_->setChannel(ftl::rgbd::kChanNone);
	}
}

const GLTexture &ftl::gui::Camera::thumbnail() {
	return GLTexture();
}

const GLTexture &ftl::gui::Camera::captureFrame() {
	float now = (float)glfwGetTime();
	delta_ = now - ftime_;
	ftime_ = now;

	if (src_ && src_->isReady()) {
		cv::Mat rgb, depth;

		// Lerp the Eye
		eye_[0] += (neye_[0] - eye_[0]) * lerpSpeed_ * delta_;
		eye_[1] += (neye_[1] - eye_[1]) * lerpSpeed_ * delta_;
		eye_[2] += (neye_[2] - eye_[2]) * lerpSpeed_ * delta_;

		Eigen::Translation3d trans(eye_);
		Eigen::Affine3d t(trans);
		Eigen::Matrix4d viewPose = t.matrix() * rotmat_;

		if (src_->hasCapabilities(ftl::rgbd::kCapMovable)) src_->setPose(viewPose);
		src_->grab();
		src_->getFrames(rgb, depth);

		if (channel_ == ftl::rgbd::kChanDeviation) {
			if (!stats_ && depth.rows > 0) {
				stats_ = new StatisticsImage(depth.size());
			}
			
			if (stats_ && depth.rows > 0) { stats_->update(depth); }
		}

		cv::Mat tmp;

		switch(channel_) {
			case ftl::rgbd::kChanDepth:
				if (depth.rows == 0) { break; }
				//imageSize = Vector2f(depth.cols,depth.rows);
				depth.convertTo(tmp, CV_8U, 255.0f / 5.0f);
				tmp = 255 - tmp;
				applyColorMap(tmp, tmp, cv::COLORMAP_JET);
				texture_.update(tmp);
				break;
			
			case ftl::rgbd::kChanDeviation:
				if (depth.rows == 0) { break; }
				//imageSize = Vector2f(depth.cols, depth.rows);
				stats_->getStdDev(tmp);
				tmp.convertTo(tmp, CV_8U, 1000.0);
				applyColorMap(tmp, tmp, cv::COLORMAP_HOT);
				texture_.update(tmp);
				break;

			case ftl::rgbd::kChanRight:
				if (depth.rows == 0 || depth.type() != CV_8UC3) { break; }
				texture_.update(depth);
				break;

			default:
				if (rgb.rows == 0) { break; }
				//imageSize = Vector2f(rgb.cols,rgb.rows);
				texture_.update(rgb);
		}
	}

	return texture_;
}

nlohmann::json ftl::gui::Camera::getMetaData() {
	return nlohmann::json();
}
