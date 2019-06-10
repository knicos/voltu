#include "src_window.hpp"

#include <nanogui/imageview.h>
#include <nanogui/combobox.h>
#include <nanogui/label.h>
#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/layout.h>

#ifdef HAVE_LIBARCHIVE
#include "ftl/rgbd/snapshot.hpp"
#endif

using ftl::gui::SourceWindow;
using ftl::rgbd::Source;
using std::string;

class GLTexture {
	public:
	GLTexture() {
		glGenTextures(1, &glid_);
        glBindTexture(GL_TEXTURE_2D, glid_);
		cv::Mat m(cv::Size(100,100), CV_8UC3);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, m.cols, m.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, m.data);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	}
	~GLTexture() {
		glDeleteTextures(1, &glid_);
	}

	void update(cv::Mat &m) {
		if (m.rows == 0) return;
		glBindTexture(GL_TEXTURE_2D, glid_);
		// TODO Allow for other formats
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, m.cols, m.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, m.data);
		auto err = glGetError();
		if (err != 0) LOG(ERROR) << "OpenGL Texture error: " << err;
	}

	unsigned int texture() const { return glid_; }

	private:
	unsigned int glid_;
};

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

class VirtualCameraView : public nanogui::ImageView {
	public:
	VirtualCameraView(nanogui::Widget *parent) : nanogui::ImageView(parent, 0) {
		src_ = nullptr;
		eye_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		centre_ = Eigen::Vector3f(0.0f, 0.0f, -4.0f);
		up_ = Eigen::Vector3f(0,1.0f,0);
		lookPoint_ = Eigen::Vector3f(0.0f,0.0f,-4.0f);
		lerpSpeed_ = 0.4f;
		depth_ = false;
	}

	void setSource(Source *src) { src_ = src; }

	bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) {
		//LOG(INFO) << "Mouse move: " << p[0];
		if (src_ && down) {
			Eigen::Vector4f camPos = src_->point(p[0],p[1]);
			camPos *= -1.0f;
			Eigen::Vector4f worldPos =  src_->getPose() * camPos;
			lookPoint_ = Eigen::Vector3f(worldPos[0],worldPos[1],worldPos[2]);
			LOG(INFO) << "Depth at click = " << -camPos[2];
		}
	}

	bool keyboardEvent(int key, int scancode, int action, int modifiers) {
		LOG(INFO) << "Key press" << key << " - " << action;
		if (key == 81 || key == 83) {
			// TODO Should rotate around lookAt object, but requires correct depth
			Eigen::Quaternion<float> q;  q = Eigen::AngleAxis<float>((key == 81) ? 0.01f : -0.01f, up_);
			eye_ = (q * (eye_ - centre_)) + centre_;
		} else if (key == 84 || key == 82) {
			float scalar = (key == 84) ? 0.99f : 1.01f;
			eye_ = ((eye_ - centre_) * scalar) + centre_;
		}
	}

	void draw(NVGcontext *ctx) {
		//net_->broadcast("grab");
		if (src_) {
			cv::Mat rgb, depth;
			centre_ += (lookPoint_ - centre_) * (lerpSpeed_ * 0.1f);
			Eigen::Matrix4f viewPose = lookAt<float>(eye_,centre_,up_).inverse();

			src_->setPose(viewPose);
			src_->grab();
			src_->getFrames(rgb, depth);

			if (depth_) {
				if (depth.rows > 0) {
					cv::Mat idepth;
					depth.convertTo(idepth, CV_8U, 255.0f / 10.0f);  // TODO(nick)
    				applyColorMap(idepth, idepth, cv::COLORMAP_JET);
					texture_.update(idepth);
					bindImage(texture_.texture());
				}
			} else {
				if (rgb.rows > 0) {
					texture_.update(rgb);
					bindImage(texture_.texture());
				}
			}

			screen()->performLayout(ctx);
		}
		ImageView::draw(ctx);
	}

	void setDepth(bool d) { depth_ = d; }

	private:
	Source *src_;
	GLTexture texture_;
	Eigen::Vector3f eye_;
	Eigen::Vector3f centre_;
	Eigen::Vector3f up_;
	Eigen::Vector3f lookPoint_;
	float lerpSpeed_;
	bool depth_;
};

SourceWindow::SourceWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl)
		: nanogui::Window(parent, "Source View"), ctrl_(ctrl) {
	setLayout(new nanogui::GroupLayout());

	using namespace nanogui;

	depth_ = false;
    src_ = ftl::create<Source>(ctrl->getRoot(), "source", ctrl->getNet());

	//Widget *tools = new Widget(this);
    //    tools->setLayout(new BoxLayout(Orientation::Horizontal,
    //                                   Alignment::Middle, 0, 6));

    new Label(this, "Select source","sans-bold");
    available_ = ctrl->getNet()->findAll<string>("list_streams");
    auto select = new ComboBox(this, available_);
    select->setCallback([this,select](int ix) {
        LOG(INFO) << "Change source: " << ix;
        src_->set("uri", available_[ix]);
    });

	ctrl->getNet()->onConnect([this,select](ftl::net::Peer *p) {
		 available_ = ctrl_->getNet()->findAll<string>("list_streams");
		 select->setItems(available_);
	});

	auto depth = new Button(this, "Depth");
	depth->setFlags(Button::ToggleButton);
	depth->setChangeCallback([this](bool state) {
		//image_->setDepth(state);
		depth_ = state;
	});

#ifdef HAVE_LIBARCHIVE
	auto snapshot = new Button(tools, "Snapshot");
	snapshot->setCallback([this] {
		try {
			char timestamp[18];
			std::time_t t=std::time(NULL);
			std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
			auto writer = ftl::rgbd::SnapshotWriter(std::string(timestamp) + ".tar.gz");
			cv::Mat rgb, depth;
			this->src_->getFrames(rgb, depth);
			if (!writer.addCameraRGBD(
					"0", // TODO
					rgb,
					depth,
					this->src_->getPose(),
					this->src_->parameters()
				)) {
				LOG(ERROR) << "Snapshot failed";
			}
		}
		catch(std::runtime_error) {
			LOG(ERROR) << "Snapshot failed (file error)";
		}
	});
#endif

	//auto imageView = new VirtualCameraView(this);
	//cam.view = imageView;
	//imageView->setGridThreshold(20);
	//imageView->setSource(src_);
	//image_ = imageView;
}

SourceWindow::~SourceWindow() {

}
