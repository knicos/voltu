#include <ftl/rgbd_display.hpp>
#include <opencv2/opencv.hpp>

using ftl::rgbd::RGBDSource;
using ftl::rgbd::Display;
using std::string;
using cv::Mat;

int Display::viewcount__ = 0;

template<class T>
Eigen::Matrix<T,4,4> lookAt
(
	Eigen::Matrix<T,3,1> const & eye,
	Eigen::Matrix<T,3,1> const & center,
	Eigen::Matrix<T,3,1> const & up
)
{
	typedef Eigen::Matrix<T,4,4> Matrix4;
	typedef Eigen::Matrix<T,3,1> Vector3;

	Vector3 f = (center - eye).normalized();
	Vector3 u = up.normalized();
	Vector3 s = f.cross(u).normalized();
	u = s.cross(f);

	Matrix4 res;
	res <<	s.x(),s.y(),s.z(),-s.dot(eye),
			u.x(),u.y(),u.z(),-u.dot(eye),
			-f.x(),-f.y(),-f.z(),f.dot(eye),
			0,0,0,1;

	return res;
}

static void setMouseAction(const std::string& winName, const MouseAction &action)
{
  cv::setMouseCallback(winName,
                       [] (int event, int x, int y, int flags, void* userdata) {
    (*(MouseAction*)userdata)(event, x, y, flags);
  }, (void*)&action);
}

Display::Display(nlohmann::json &config) : ftl::Configurable(config) {
	name_ = config.value("name", string("View [")+std::to_string(viewcount__)+string("]"));
	viewcount__++;

	init();
}

Display::Display(nlohmann::json &config, RGBDSource *source)
		: ftl::Configurable(config) {
	name_ = config.value("name", string("View [")+std::to_string(viewcount__)+string("]"));
	viewcount__++;
	init();
}

Display::~Display() {

}

void Display::init() {
	active_ = true;
	source_ = nullptr;
	cv::namedWindow(name_, cv::WINDOW_KEEPRATIO);

	eye_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	centre_ = Eigen::Vector3f(0.0f, 0.0f, -4.0f);
	up_ = Eigen::Vector3f(0,1.0f,0);
	lookPoint_ = Eigen::Vector3f(0.0f,0.0f,-4.0f);
	lerpSpeed_ = 0.4f;

	// Keyboard camera controls
	onKey([this](int key) {
		//LOG(INFO) << "Key = " << key;
		if (key == 81 || key == 83) {
			Eigen::Quaternion<float> q;  q = Eigen::AngleAxis<float>((key == 81) ? 0.01f : -0.01f, up_);
			eye_ = (q * (eye_ - centre_)) + centre_;
		} else if (key == 84 || key == 82) {
			float scalar = (key == 84) ? 0.99f : 1.01f;
			eye_ = ((eye_ - centre_) * scalar) + centre_;
		}
	});

	// TODO(Nick) Calculate "camera" properties of viewport.
	mouseaction_ = [this]( int event, int ux, int uy, int) {
		//LOG(INFO) << "Mouse " << ux << "," << uy;
		if (event == 1 && source_) {   // click
			Eigen::Vector4f camPos = source_->point(ux,uy);
			camPos *= -1.0f;
			Eigen::Vector4f worldPos =  source_->getPose() * camPos;
			lookPoint_ = Eigen::Vector3f(worldPos[0],worldPos[1],worldPos[2]);
			LOG(INFO) << "Depth at click = " << -camPos[2];
		}
	};
	::setMouseAction(name_, mouseaction_);
}

void Display::wait(int ms) {
	while (true) {
		int key = cv::waitKey(ms);

		if(key == 27) {
			// exit if ESC is pressed
			active_ = false;
		} else if (key == -1) {
			return;
		} else {
			ms = 1;
			for (auto &h : key_handlers_) {
				h(key);
			}
		}
	}
}

void Display::update() {
	if (!source_) return;

	centre_ += (lookPoint_ - centre_) * (lerpSpeed_ * 0.1f);
	Eigen::Matrix4f viewPose = lookAt<float>(eye_,centre_,up_).inverse();
	source_->setPose(viewPose);

	Mat rgb, depth;
	//source_->grab();
	source_->getRGBD(rgb, depth);
	if (rgb.rows > 0) cv::imshow(name_, rgb);
	wait(1);
}
