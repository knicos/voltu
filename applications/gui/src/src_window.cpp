#include "src_window.hpp"

#include "pose_window.hpp"

#include <nanogui/imageview.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
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

SourceWindow::SourceWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl)
		: nanogui::Window(parent, "Source View"), ctrl_(ctrl) {
	setLayout(new nanogui::GroupLayout());

	using namespace nanogui;
	
	mode_ = Mode::rgb;
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

	new Label(this, "Source Options","sans-bold");

	auto tools = new Widget(this);
    tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                       Alignment::Middle, 0, 6));

	auto button_rgb = new Button(tools, "RGB");
	button_rgb->setTooltip("RGB left image");
	button_rgb->setFlags(Button::RadioButton);
	button_rgb->setPushed(true);
	button_rgb->setChangeCallback([this](bool state) { mode_ = Mode::rgb; });

	auto button_depth = new Button(tools, "Depth");
	button_depth->setFlags(Button::RadioButton);
	button_depth->setChangeCallback([this](bool state) { mode_ = Mode::depth; });

	auto button_stddev = new Button(tools, "SD. 25");
	button_stddev->setTooltip("Standard Deviation over 25 frames");
	button_stddev->setFlags(Button::RadioButton);
	button_stddev->setChangeCallback([this](bool state) { mode_ = Mode::stddev; });

	auto button_pose = new Button(this, "Adjust Pose", ENTYPO_ICON_COMPASS);
	button_pose->setCallback([this]() {
		auto posewin = new PoseWindow(screen(), ctrl_, src_->getURI());
		posewin->setTheme(theme());
	});

#ifdef HAVE_LIBARCHIVE
	auto button_snapshot = new Button(this, "Snapshot", ENTYPO_ICON_IMAGES);
	button_snapshot->setCallback([this] {
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
