#include <ftl/configuration.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/rgbd.hpp>
#include <ftl/master.hpp>

#include <loguru.hpp>

#include <opencv2/opencv.hpp>

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/imageview.h>
#include <nanogui/combobox.h>
#include <nanogui/label.h>
#include <nanogui/toolbutton.h>

#include "ctrl_window.hpp"
#include "src_window.hpp"

using std::string;
using ftl::rgbd::Source;

using ftl::rgbd::isValidDepth;

/*struct SourceViews {
	ftl::rgbd::RGBDSource *source;
	GLTexture texture;
	nanogui::ImageView *view;
};*/

class StatisticsImage {
private:
	std::vector<float> data_;
	cv::Size size_;
	int n_;

public:
	StatisticsImage(cv::Size size) {
		size_ = size;
		data_ = std::vector<float>(size.width * size.height * 2, 0.0f);
		n_ = 0;
	}

	void update(const cv::Mat &in);
	void getStdDev(cv::Mat &out);
	void getMean(cv::Mat &out);
};

void StatisticsImage::update(const cv::Mat &in) {
	DCHECK(in.type() == CV_32F);
	DCHECK(in.size() == size_);
	// Welford's Method

	n_++;
	for (int i = 0; i < size_.width * size_.height; i++) {
		float x = ((float*) in.data)[i];
		if (!isValidDepth(x)) { continue; } // invalid value
		float &m = data_[2*i];
		float &S = data_[2*i+1];
		float m_prev = m;
		m = m + (x - m) / n_;
		S = S + (x - m) * (x - m_prev);
	}
}

void StatisticsImage::getStdDev(cv::Mat &in) {
	in = cv::Mat(size_, CV_32F, 0.0f);

	for (int i = 0; i < size_.width * size_.height; i++) {
		float &m = data_[2*i];
		float &S = data_[2*i+1];
		((float*) in.data)[i] = sqrt(S / n_);
	}
}

void StatisticsImage::getMean(cv::Mat &in) {
	in = cv::Mat(size_, CV_32F, 0.0f);

	for (int i = 0; i < size_.width * size_.height; i++) {
		((float*) in.data)[i] = data_[2*i];
	}
}

class StatisticsImageNSamples {
private:
	std::vector<cv::Mat> samples_;
	cv::Size size_;
	int i_;
	int n_;

public:
	StatisticsImageNSamples(cv::Size size, int n) {
		size_ = size;
		samples_ = std::vector<cv::Mat>(n);
		i_ = 0;
		n_ = n;
	}

	void update(const cv::Mat &in);
	void getStdDev(cv::Mat &out);
	void getVariance(cv::Mat &out);
	void getMean(cv::Mat &out);
};

void StatisticsImageNSamples::update(const cv::Mat &in) {
	DCHECK(in.type() == CV_32F);
	DCHECK(in.size() == size_);

	i_ = (i_ + 1) % n_;
	in.copyTo(samples_[i_]);
}

void StatisticsImageNSamples::getStdDev(cv::Mat &out) {
	cv::Mat var;
	getVariance(var);
	cv::sqrt(var, out);
}

void StatisticsImageNSamples::getVariance(cv::Mat &out) {
	// Welford's Method
	cv::Mat mat_m(size_, CV_32F, 0.0f);
	cv::Mat mat_S(size_, CV_32F, 0.0f);

	float n = 0.0f;
	for (int i_sample = (i_ + 1) % n_; i_sample != i_; i_sample = (i_sample + 1) % n_) {
		if (samples_[i_sample].size() != size_) continue;
		n += 1.0f;
		for (int i = 0; i < size_.width * size_.height; i++) {
			float &x = ((float*) samples_[i_sample].data)[i];
			float &m = ((float*) mat_m.data)[i];
			float &S = ((float*) mat_S.data)[i];
			float m_prev = m;

			if (!isValidDepth(x)) continue;

			m = m + (x - m) / n;
			S = S + (x - m) * (x - m_prev);
		}
	}

	mat_S.copyTo(out);
}

void StatisticsImageNSamples::getMean(cv::Mat &in) {}

namespace {
    constexpr char const *const defaultImageViewVertexShader =
        R"(#version 330
        uniform vec2 scaleFactor;
        uniform vec2 position;
        in vec2 vertex;
        out vec2 uv;
        void main() {
            uv = vertex;
            vec2 scaledVertex = (vertex * scaleFactor) + position;
            gl_Position  = vec4(2.0*scaledVertex.x - 1.0,
                                1.0 - 2.0*scaledVertex.y,
                                0.0, 1.0);
        })";

    constexpr char const *const defaultImageViewFragmentShader =
        R"(#version 330
        uniform sampler2D image;
        out vec4 color;
        in vec2 uv;
        void main() {
            color = texture(image, uv);
        })";


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
}


static Eigen::Affine3f create_rotation_matrix(float ax, float ay, float az) {
  Eigen::Affine3f rx =
      Eigen::Affine3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
  Eigen::Affine3f ry =
      Eigen::Affine3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
  Eigen::Affine3f rz =
      Eigen::Affine3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
  return ry * rz * rx;
}


class FTLApplication : public nanogui::Screen {
	public:
	explicit FTLApplication(ftl::Configurable *root, ftl::net::Universe *net, ftl::ctrl::Master *controller) : nanogui::Screen(Eigen::Vector2i(1024, 768), "FT-Lab Remote Presence") {
		using namespace nanogui;
		net_ = net;
		ctrl_ = controller;

		status_ = "FT-Lab Remote Presence System";

		setSize(Vector2i(1280,720));

		Theme *toolbuttheme = new Theme(*theme());
		toolbuttheme->mBorderDark = nanogui::Color(0,0);
		toolbuttheme->mBorderLight = nanogui::Color(0,0);
		toolbuttheme->mButtonGradientBotFocused = nanogui::Color(60,255);
		toolbuttheme->mButtonGradientBotUnfocused = nanogui::Color(0,0);
		toolbuttheme->mButtonGradientTopFocused = nanogui::Color(60,255);
		toolbuttheme->mButtonGradientTopUnfocused = nanogui::Color(0,0);
		toolbuttheme->mButtonGradientTopPushed = nanogui::Color(60,180);
		toolbuttheme->mButtonGradientBotPushed = nanogui::Color(60,180);

		Theme *windowtheme = new Theme(*theme());
		windowtheme->mWindowFillFocused = nanogui::Color(220, 200);
		windowtheme->mWindowFillUnfocused = nanogui::Color(220, 200);
		windowtheme->mWindowHeaderGradientBot = nanogui::Color(60,230);
		windowtheme->mWindowHeaderGradientTop = nanogui::Color(60,230);
		windowtheme->mTextColor = nanogui::Color(20,255);
		windowtheme->mWindowCornerRadius = 2;
		windowtheme->mButtonGradientBotFocused = nanogui::Color(210,255);
		windowtheme->mButtonGradientBotUnfocused = nanogui::Color(190,255);
		windowtheme->mButtonGradientTopFocused = nanogui::Color(230,255);
		windowtheme->mButtonGradientTopUnfocused = nanogui::Color(230,255);
		windowtheme->mButtonGradientTopPushed = nanogui::Color(170,255);
		windowtheme->mButtonGradientBotPushed = nanogui::Color(210,255);
		windowtheme->mBorderDark = nanogui::Color(150,255);
		windowtheme->mBorderMedium = nanogui::Color(165,255);
		windowtheme->mBorderLight = nanogui::Color(230,255);
		windowtheme->mButtonFontSize = 16;
		windowtheme->mTextColorShadow = nanogui::Color(0,0);
		windowtheme->mWindowTitleUnfocused = windowtheme->mWindowTitleFocused;
		windowtheme->mWindowTitleFocused = nanogui::Color(240,255);
		windowtheme->mIconScale = 0.85f;

		auto toolbar = new Window(this, "");
		toolbar->setPosition(Vector2i(0,0));
		toolbar->setFixedWidth(50);
		toolbar->setFixedHeight(height());
		//toolbar->setLayout(new BoxLayout(Orientation::Vertical,
        //                               Alignment::Middle, 0, 10));

		setResizeCallback([this,toolbar](Vector2i s) {
			toolbar->setFixedHeight(s[1]);
		});

		auto innertool = new Widget(toolbar);
		innertool->setLayout(new BoxLayout(Orientation::Vertical,
                                       Alignment::Middle, 0, 10));
		innertool->setPosition(Vector2i(5,10));

		// Padding widget
		//auto w = new Widget(innertool);
		//w->setHeight(10);

		auto button = new ToolButton(innertool, ENTYPO_ICON_HOME);
		button->setIconExtraScale(1.5f);
		button->setTheme(toolbuttheme);
		button->setTooltip("Home");
		button->setFixedSize(Vector2i(40,40));
		button->setCallback([this]() {
			//swindow_->setVisible(true);
		});

		button = new ToolButton(innertool, ENTYPO_ICON_EDIT);
		button->setIconExtraScale(1.5f);
		button->setTheme(toolbuttheme);
		button->setTooltip("Edit Scene");
		button->setFixedSize(Vector2i(40,40));
		button->setCallback([this]() {
			//swindow_->setVisible(true);
		});

		button = new ToolButton(innertool, ENTYPO_ICON_CAMERA);
		button->setIconExtraScale(1.5f);
		button->setTheme(toolbuttheme);
		button->setTooltip("Camera Sources");
		button->setFixedSize(Vector2i(40,40));
		button->setCallback([this]() {
			swindow_->setVisible(true);
		});

		button = new ToolButton(innertool, ENTYPO_ICON_SIGNAL);
		button->setIconExtraScale(1.5f);
		button->setTheme(toolbuttheme);
		button->setTooltip("Connections");
		button->setFixedSize(Vector2i(40,40));
		button->setCallback([this]() {
			cwindow_->setVisible(true);
		});

		button = new ToolButton(toolbar, ENTYPO_ICON_COG);
		button->setIconExtraScale(1.5f);
		button->setTheme(toolbuttheme);
		button->setTooltip("Settings");
		button->setFixedSize(Vector2i(40,40));
		button->setPosition(Vector2i(5,height()-50));

		//configwindow_ = new ConfigWindow(parent, ctrl_);
		cwindow_ = new ftl::gui::ControlWindow(this, controller);
		swindow_ = new ftl::gui::SourceWindow(this, controller);

		cwindow_->setPosition(Eigen::Vector2i(80, 20));
		swindow_->setPosition(Eigen::Vector2i(80, 400));
		cwindow_->setVisible(false);
		swindow_->setVisible(false);
		cwindow_->setTheme(windowtheme);
		swindow_->setTheme(windowtheme);

		//src_ = nullptr;
		eye_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		neye_ = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
		orientation_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		up_ = Eigen::Vector3f(0,1.0f,0);
		//lookPoint_ = Eigen::Vector3f(0.0f,0.0f,-4.0f);
		lerpSpeed_ = 0.999f;
		depth_ = false;
		ftime_ = (float)glfwGetTime();

		mShader.init("RGBDShader", defaultImageViewVertexShader,
                 defaultImageViewFragmentShader);

		MatrixXu indices(3, 2);
		indices.col(0) << 0, 1, 2;
		indices.col(1) << 2, 3, 1;

		MatrixXf vertices(2, 4);
		vertices.col(0) << 0, 0;
		vertices.col(1) << 1, 0;
		vertices.col(2) << 0, 1;
		vertices.col(3) << 1, 1;

		mShader.bind();
		mShader.uploadIndices(indices);
		mShader.uploadAttrib("vertex", vertices);

		setVisible(true);
		performLayout();
	}

	~FTLApplication() {
		mShader.free();
	}

	bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) {
		if (Screen::mouseMotionEvent(p, rel, button, modifiers)) {
			return true;
		} else {
			if (button == 1) {
				orientation_[0] += ((float)rel[1] * 0.2f * delta_);
				//orientation_[2] += std::cos(orientation_[1])*((float)rel[1] * 0.2f * delta_);
				orientation_[1] -= (float)rel[0] * 0.2f * delta_;
			}
		}
	}

	bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) {
		if (Screen::mouseButtonEvent(p, button, down, modifiers)) {
			return true;
		} else {
			auto src_ = swindow_->getSource();
			if (src_ && src_->isReady() && down) {
				Eigen::Vector2f screenSize = size().cast<float>();
				auto mScale = (screenSize.cwiseQuotient(imageSize).minCoeff());
				Eigen::Vector2f scaleFactor = mScale * imageSize.cwiseQuotient(screenSize);
				Eigen::Vector2f positionInScreen(0.0f, 0.0f);
				auto mOffset = (screenSize - (screenSize.cwiseProduct(scaleFactor))) / 2;
				Eigen::Vector2f positionAfterOffset = positionInScreen + mOffset;

				float sx = ((float)p[0] - positionAfterOffset[0]) / mScale;
				float sy = ((float)p[1] - positionAfterOffset[1]) / mScale;

				Eigen::Vector4f camPos;

				try {
					camPos = src_->point(sx,sy).cast<float>();
				} catch(...) {
					return true;
				}
				
				camPos *= -1.0f;
				Eigen::Vector4f worldPos =  src_->getPose().cast<float>() * camPos;
				//lookPoint_ = Eigen::Vector3f(worldPos[0],worldPos[1],worldPos[2]);
				LOG(INFO) << "Depth at click = " << -camPos[2];
				return true;
			}
		return false;
		}
	}

	bool keyboardEvent(int key, int scancode, int action, int modifiers) {
		using namespace Eigen;
		if (Screen::keyboardEvent(key, scancode, action, modifiers)) {
			return true;
		} else {
			LOG(INFO) << "Key press " << key << " - " << action << " - " << modifiers;
			if (key == 263 || key == 262) {
				float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
				float scalar = (key == 263) ? -mag : mag;
				Eigen::Affine3f r = create_rotation_matrix(orientation_[0], orientation_[1], orientation_[2]);
				neye_ += r.matrix()*Vector4f(scalar,0.0,0.0,1.0);
				return true;
			} else if (key == 264 || key == 265) {
				float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
				float scalar = (key == 264) ? -mag : mag;
				Eigen::Affine3f r = create_rotation_matrix(orientation_[0], orientation_[1], orientation_[2]);
				neye_ += r.matrix()*Vector4f(0.0,0.0,scalar,1.0);
				return true;
			} else if (key == 266 || key == 267) {
				float mag = (modifiers & 0x1) ? 0.01f : 0.1f;
				float scalar = (key == 266) ? -mag : mag;
				Eigen::Affine3f r = create_rotation_matrix(orientation_[0], orientation_[1], orientation_[2]);
				neye_ += r.matrix()*Vector4f(0.0,scalar,0.0,1.0);
				return true;
			} else if (action == 1 && key == 'H') {
				swindow_->setVisible(false);
				cwindow_->setVisible(false);
			} else if (action == 1 && key == 32) {
				ctrl_->pause();
			}
			return false;
		}
	}

	StatisticsImageNSamples *stats_ = nullptr;

	virtual void draw(NVGcontext *ctx) {
		using namespace Eigen;

		auto src_ = swindow_->getSource();
		imageSize = {0, 0};

		float now = (float)glfwGetTime();
		delta_ = now - ftime_;
		ftime_ = now;

		if (src_) {
			cv::Mat rgb, depth;

			// Lerp the Eye
			eye_[0] += (neye_[0] - eye_[0]) * lerpSpeed_ * delta_;
			eye_[1] += (neye_[1] - eye_[1]) * lerpSpeed_ * delta_;
			eye_[2] += (neye_[2] - eye_[2]) * lerpSpeed_ * delta_;

			Eigen::Affine3f r = create_rotation_matrix(orientation_[0], orientation_[1], orientation_[2]);
			Eigen::Translation3f trans(eye_);
			Eigen::Affine3f t(trans);
			Eigen::Matrix4f viewPose = (t * r).matrix();

			src_->setPose(viewPose.cast<double>());
			src_->grab();
			src_->getFrames(rgb, depth);

			if (!stats_ && depth.rows > 0) {
				stats_ = new StatisticsImageNSamples(depth.size(), 25);
			}
			
			if (stats_ && depth.rows > 0) { stats_->update(depth); }

			cv::Mat tmp;

			using ftl::gui::SourceWindow;
			switch(swindow_->getMode()) {
				case SourceWindow::Mode::depth:
					if (depth.rows == 0) { break; }
					imageSize = Vector2f(depth.cols,depth.rows);
					depth.convertTo(tmp, CV_8U, 255.0f / 5.0f);
					tmp = 255 - tmp;
					applyColorMap(tmp, tmp, cv::COLORMAP_JET);
					texture_.update(tmp);
					mImageID = texture_.texture();
					break;
				
				case SourceWindow::Mode::stddev:
					if (depth.rows == 0) { break; }
					imageSize = Vector2f(depth.cols, depth.rows);
					stats_->getStdDev(tmp);
					tmp.convertTo(tmp, CV_8U, 50.0);
					applyColorMap(tmp, tmp, cv::COLORMAP_HOT);
					texture_.update(tmp);
					mImageID = texture_.texture();
					break;

				default:
					if (rgb.rows == 0) { break; }
					imageSize = Vector2f(rgb.cols,rgb.rows);
					texture_.update(rgb);
					mImageID = texture_.texture();
			}
		}

		Vector2f screenSize = size().cast<float>();

		if (imageSize[0] > 0) {
			auto mScale = (screenSize.cwiseQuotient(imageSize).minCoeff());
			Vector2f scaleFactor = mScale * imageSize.cwiseQuotient(screenSize);
			Vector2f positionInScreen(0.0f, 0.0f);
			auto mOffset = (screenSize - (screenSize.cwiseProduct(scaleFactor))) / 2;
			Vector2f positionAfterOffset = positionInScreen + mOffset;
			Vector2f imagePosition = positionAfterOffset.cwiseQuotient(screenSize);
			//glEnable(GL_SCISSOR_TEST);
			//float r = screen->pixelRatio();
			/* glScissor(positionInScreen.x() * r,
					(screenSize.y() - positionInScreen.y() - size().y()) * r,
					size().x() * r, size().y() * r);*/
			mShader.bind();
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, mImageID);
			mShader.setUniform("image", 0);
			mShader.setUniform("scaleFactor", scaleFactor);
			mShader.setUniform("position", imagePosition);
			mShader.drawIndexed(GL_TRIANGLES, 0, 2);
			//glDisable(GL_SCISSOR_TEST);
		}

		nvgTextAlign(ctx, NVG_ALIGN_RIGHT);
		nvgText(ctx, screenSize[0]-10, screenSize[1]-20, status_.c_str(), NULL);

		/* Draw the user interface */
		screen()->performLayout(ctx);
		Screen::draw(ctx);
	}

	private:
	ftl::gui::SourceWindow *swindow_;
	ftl::gui::ControlWindow *cwindow_;
	//std::vector<SourceViews> sources_;
	ftl::net::Universe *net_;
	nanogui::GLShader mShader;
    GLuint mImageID;
	//Source *src_;
	GLTexture texture_;
	Eigen::Vector3f eye_;
	Eigen::Vector4f neye_;
	Eigen::Vector3f orientation_;
	Eigen::Vector3f up_;
	//Eigen::Vector3f lookPoint_;
	float lerpSpeed_;
	bool depth_;
	float ftime_;
	float delta_;
	Eigen::Vector2f imageSize;
	ftl::ctrl::Master *ctrl_;
	std::string status_;
};

int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "gui_default");
	ftl::net::Universe *net = ftl::create<ftl::net::Universe>(root, "net");

	net->start();
	//net->waitConnections();

	ftl::ctrl::Master *controller = new ftl::ctrl::Master(root, net);
	controller->onLog([](const ftl::ctrl::LogEvent &e){
		const int v = e.verbosity;
		switch (v) {
		case -2:	LOG(ERROR) << "Remote log: " << e.message; break;
		case -1:	LOG(WARNING) << "Remote log: " << e.message; break;
		case 0:		LOG(INFO) << "Remote log: " << e.message; break;
		}
	});

	/*auto available = net.findAll<string>("list_streams");
	for (auto &a : available) {
		std::cout << " -- " << a << std::endl;
	}*/

	try {
		nanogui::init();

		/* scoped variables */ {
			nanogui::ref<FTLApplication> app = new FTLApplication(root, net, controller);
			app->drawAll();
			app->setVisible(true);
			nanogui::mainloop();
		}

		nanogui::shutdown();
	} catch (const std::runtime_error &e) {
		std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
		#if defined(_WIN32)
			MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
		#else
			std::cerr << error_msg << std::endl;
		#endif
		return -1;
	}

	net->shutdown();
	delete controller;
	delete net;
	delete root;

	return 0;
}