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

#include "ctrl_window.hpp"
#include "src_window.hpp"

using std::string;
using ftl::rgbd::Source;

/*struct SourceViews {
	ftl::rgbd::RGBDSource *source;
	GLTexture texture;
	nanogui::ImageView *view;
};*/


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


class FTLApplication : public nanogui::Screen {
	public:
	explicit FTLApplication(ftl::Configurable *root, ftl::net::Universe *net, ftl::ctrl::Master *controller) : nanogui::Screen(Eigen::Vector2i(1024, 768), "FT-Lab GUI") {
		using namespace nanogui;
		net_ = net;

		cwindow_ = new ftl::gui::ControlWindow(this, controller);
		swindow_ = new ftl::gui::SourceWindow(this, controller);

		//src_ = nullptr;
		eye_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		centre_ = Eigen::Vector3f(0.0f, 0.0f, -4.0f);
		up_ = Eigen::Vector3f(0,1.0f,0);
		lookPoint_ = Eigen::Vector3f(0.0f,0.0f,-4.0f);
		lerpSpeed_ = 0.4f;
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
					camPos = src_->point(sx,sy);
				} catch(...) {
					return true;
				}
				
				camPos *= -1.0f;
				Eigen::Vector4f worldPos =  src_->getPose() * camPos;
				lookPoint_ = Eigen::Vector3f(worldPos[0],worldPos[1],worldPos[2]);
				LOG(INFO) << "Depth at click = " << -camPos[2];
				return true;
			}
		return false;
		}
	}

	bool keyboardEvent(int key, int scancode, int action, int modifiers) {
		if (Screen::keyboardEvent(key, scancode, action, modifiers)) {
			return true;
		} else {
			LOG(INFO) << "Key press " << key << " - " << action;
			if (key == 263 || key == 262) {
				// TODO Should rotate around lookAt object, but requires correct depth
				Eigen::Quaternion<float> q;  q = Eigen::AngleAxis<float>((key == 262) ? 0.01f : -0.01f, up_);
				eye_ = (q * (eye_ - centre_)) + centre_;
				return true;
			} else if (key == 264 || key == 265) {
				float scalar = (key == 264) ? 0.99f : 1.01f;
				eye_ = ((eye_ - centre_) * scalar) + centre_;
				return true;
			} else if (action == 1 && key == 'H') {
				swindow_->setVisible(!swindow_->visible());
				cwindow_->setVisible(!cwindow_->visible());
			}
			return false;
		}
	}

	virtual void draw(NVGcontext *ctx) {
		using namespace Eigen;

		auto src_ = swindow_->getSource();
		imageSize = {0, 0};

		float now = (float)glfwGetTime();
		float delta = now - ftime_;
		ftime_ = now;

		if (src_) {
			cv::Mat rgb, depth;
			centre_ += (lookPoint_ - centre_) * (lerpSpeed_ * delta);
			Eigen::Matrix4f viewPose = lookAt<float>(eye_,centre_,up_).inverse();

			src_->setPose(viewPose);
			src_->grab();
			src_->getFrames(rgb, depth);

			if (swindow_->getDepth()) {
				if (depth.rows > 0) {
					imageSize = Vector2f(depth.cols,depth.rows);
					cv::Mat idepth;
					depth.convertTo(idepth, CV_8U, 255.0f / 10.0f);  // TODO(nick)
    				applyColorMap(idepth, idepth, cv::COLORMAP_JET);
					texture_.update(idepth);
					mImageID = texture_.texture();
				}
			} else {
				if (rgb.rows > 0) {
					imageSize = Vector2f(rgb.cols,rgb.rows);
					texture_.update(rgb);
					mImageID = texture_.texture();
				}
			}
		}

		if (imageSize[0] > 0) {
			Vector2f screenSize = size().cast<float>();
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

		nvgText(ctx, 10, 20, "FT-Lab Remote Presence System", NULL);

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
	Eigen::Vector3f centre_;
	Eigen::Vector3f up_;
	Eigen::Vector3f lookPoint_;
	float lerpSpeed_;
	bool depth_;
	float ftime_;
	Eigen::Vector2f imageSize;
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