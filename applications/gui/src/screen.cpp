#include "screen.hpp"

#include <ftl/streams/netstream.hpp>
#include <ftl/rgbd/frameset.hpp>

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/imageview.h>
#include <nanogui/label.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>

#include <sstream>

#include <nlohmann/json.hpp>

#include <loguru.hpp>

#include <opencv2/core/eigen.hpp>

#include "ctrl_window.hpp"
#include "src_window.hpp"
#include "config_window.hpp"
#include "camera.hpp"
#include "media_panel.hpp"

#ifdef HAVE_OPENVR
#include "vr.hpp"
#endif

using ftl::gui::Screen;
using ftl::gui::Camera;
using std::string;
using ftl::rgbd::Source;
using ftl::rgbd::isValidDepth;

namespace {
	constexpr char const *const defaultImageViewVertexShader =
		R"(#version 330
		uniform vec2 scaleFactor;
		uniform vec2 position;
		in vec2 vertex;
		out vec2 uv;
		void main() {
			uv = vec2(vertex.x, vertex.y);
			vec2 scaledVertex = (vertex * scaleFactor) + position;
			gl_Position  = vec4(2.0*scaledVertex.x - 1.0,
								2.0*scaledVertex.y - 1.0,
								0.0, 1.0);
		})";

	constexpr char const *const defaultImageViewFragmentShader =
		R"(#version 330
		uniform sampler2D image1;
		uniform sampler2D image2;
		uniform sampler2D depthImage;
		uniform float blendAmount;
		out vec4 color;
		in vec2 uv;
		void main() {
			color = blendAmount * texture(image1, uv) + (1.0 - blendAmount) * texture(image2, uv);
			color.w = 1.0f;
			gl_FragDepth = texture(depthImage, uv).r;
		})";
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

ftl::gui::Screen::Screen(ftl::Configurable *proot, ftl::net::Universe *pnet, ftl::ctrl::Master *controller) :
		nanogui::Screen(Eigen::Vector2i(1024, 768), "FT-Lab Remote Presence"),
		status_("FT-Lab Remote Presence System") {
	using namespace nanogui;
	net_ = pnet;
	ctrl_ = controller;
	root_ = proot;
	camera_ = nullptr;
	last_stats_count_ = 0;

	#ifdef HAVE_OPENVR
	HMD_ = nullptr;
	#endif

	zoom_ = root_->value("zoom", 1.0f);
	root_->on("zoom", [this](const ftl::config::Event &e) {
		zoom_ = root_->value("zoom", 1.0f);
	});

	pos_x_ = root_->value("position_x", 0.0f);
	root_->on("position_x", [this](const ftl::config::Event &e) {
		pos_x_ = root_->value("position_x", 0.0f);
	});
	pos_y_ = root_->value("position_y", 0.0f);
	root_->on("position_y", [this](const ftl::config::Event &e) {
		pos_y_ = root_->value("position_y", 0.0f);
	});

	shortcuts_ = ftl::create<ftl::Configurable>(root_, "shortcuts");

	setSize(Vector2i(1280,720));

	toolbuttheme = new Theme(*theme());
	toolbuttheme->mBorderDark = nanogui::Color(0,0);
	toolbuttheme->mBorderLight = nanogui::Color(0,0);
	toolbuttheme->mButtonGradientBotFocused = nanogui::Color(60,255);
	toolbuttheme->mButtonGradientBotUnfocused = nanogui::Color(0,0);
	toolbuttheme->mButtonGradientTopFocused = nanogui::Color(60,255);
	toolbuttheme->mButtonGradientTopUnfocused = nanogui::Color(0,0);
	toolbuttheme->mButtonGradientTopPushed = nanogui::Color(60,180);
	toolbuttheme->mButtonGradientBotPushed = nanogui::Color(60,180);
	toolbuttheme->mTextColor = nanogui::Color(0.9f,0.9f,0.9f,0.9f);

	mediatheme = new Theme(*theme());
	mediatheme->mIconScale = 1.2f;
	mediatheme->mWindowDropShadowSize = 0;
	mediatheme->mWindowFillFocused = nanogui::Color(45, 150);
	mediatheme->mWindowFillUnfocused = nanogui::Color(45, 80);
	mediatheme->mButtonGradientTopUnfocused = nanogui::Color(0,0);
	mediatheme->mButtonGradientBotUnfocused = nanogui::Color(0,0);
	mediatheme->mButtonGradientTopFocused = nanogui::Color(80,230);
	mediatheme->mButtonGradientBotFocused = nanogui::Color(80,230);
	mediatheme->mIconColor = nanogui::Color(255,255);
	mediatheme->mTextColor = nanogui::Color(1.0f,1.0f,1.0f,1.0f);
	mediatheme->mBorderDark = nanogui::Color(0,0);
	mediatheme->mBorderMedium = nanogui::Color(0,0);
	mediatheme->mBorderLight = nanogui::Color(0,0);
	mediatheme->mDropShadow = nanogui::Color(0,0);
	mediatheme->mButtonFontSize = 30;
	mediatheme->mStandardFontSize = 20;

	windowtheme = new Theme(*theme());
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
		mwindow_->setPosition(Vector2i(s[0] / 2 - mwindow_->width()/2, s[1] - 30 - mwindow_->height()));
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
		setActiveCamera(nullptr);
	});

	/*button = new ToolButton(innertool, ENTYPO_ICON_PLUS);
	button->setIconExtraScale(1.5f);
	button->setTheme(toolbuttheme);
	button->setTooltip("Add new");
	button->setFixedSize(Vector2i(40,40));
	button->setCallback([this]() {
		//swindow_->setVisible(true);
	});*/

	auto popbutton = new PopupButton(innertool, "", ENTYPO_ICON_PLUS);
	popbutton->setIconExtraScale(1.5f);
	popbutton->setTheme(toolbuttheme);
	popbutton->setTooltip("Add");
	popbutton->setFixedSize(Vector2i(40,40));
	popbutton->setSide(Popup::Side::Right);
	popbutton->setChevronIcon(0);
	Popup *popup = popbutton->popup();
	popup->setLayout(new GroupLayout());
	popup->setTheme(toolbuttheme);
	//popup->setAnchorHeight(100);

	auto itembutton = new Button(popup, "Add Camera", ENTYPO_ICON_CAMERA);
	itembutton->setCallback([this,popup]() {
		swindow_->setVisible(true);
		popup->setVisible(false);
	});

	itembutton = new Button(popup, "Add Node", ENTYPO_ICON_LAPTOP);
	itembutton->setCallback([this,popup]() {
		cwindow_->setVisible(true);
		popup->setVisible(false);
	});

	popbutton = new PopupButton(innertool, "", ENTYPO_ICON_TOOLS);
	popbutton->setIconExtraScale(1.5f);
	popbutton->setTheme(toolbuttheme);
	popbutton->setTooltip("Tools");
	popbutton->setFixedSize(Vector2i(40,40));
	popbutton->setSide(Popup::Side::Right);
	popbutton->setChevronIcon(0);
	popup = popbutton->popup();
	popup->setLayout(new GroupLayout());
	popup->setTheme(toolbuttheme);
	//popbutton->setCallback([this]() {
	//	cwindow_->setVisible(true);
	//});

	itembutton = new Button(popup, "Connections");
	itembutton->setCallback([this,popup]() {
		cwindow_->setVisible(true);
		popup->setVisible(false);
	});

	itembutton = new Button(popup, "Manual Registration");
	itembutton->setCallback([this,popup]() {
		// Show pose win...
		popup->setVisible(false);
	});

	itembutton = new Button(innertool, "", ENTYPO_ICON_COG);
	itembutton->setIconExtraScale(1.5f);
	itembutton->setTheme(toolbuttheme);
	itembutton->setTooltip("Settings");
	itembutton->setFixedSize(Vector2i(40,40));

	itembutton->setCallback([this]() {
		auto config_window = new ConfigWindow(this, ctrl_);
		config_window->setTheme(windowtheme);
	});

	/*
	//net_->onConnect([this,popup](ftl::net::Peer *p) {
	{
		LOG(INFO) << "NET CONNECT";
		auto node_details = ctrl_->getControllers();

		for (auto &d : node_details) {
			LOG(INFO) << "ADDING TITLE: " << d.dump();
			auto peer = ftl::UUID(d["id"].get<std::string>());
			auto itembutton = new Button(popup, d["title"].get<std::string>());
			itembutton->setCallback([this,popup,peer]() {
				auto config_window = new ConfigWindow(this, ctrl_);
				config_window->setTheme(windowtheme);
			});
		}
	}
	//});

	itembutton = new Button(popup, "Local");
	itembutton->setCallback([this,popup]() {
		auto config_window = new ConfigWindow(this, ctrl_);
		config_window->setTheme(windowtheme);
	});
	*/

	//configwindow_ = new ConfigWindow(parent, ctrl_);
	//cwindow_ = new ftl::gui::ControlWindow(this, controller);
	swindow_ = new ftl::gui::SourceWindow(this);
	mwindow_ = new ftl::gui::MediaPanel(this, swindow_);
	mwindow_->setVisible(false);
	mwindow_->setTheme(mediatheme);

	//cwindow_->setPosition(Eigen::Vector2i(80, 20));
	//swindow_->setPosition(Eigen::Vector2i(80, 400));
	//cwindow_->setVisible(false);
	swindow_->setVisible(true);
	swindow_->center();
	//cwindow_->setTheme(windowtheme);
	swindow_->setTheme(mediatheme);

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

#ifdef HAVE_OPENVR
bool ftl::gui::Screen::initVR() {
	if (!vr::VR_IsHmdPresent()) {
		return false;
	}

	vr::EVRInitError eError = vr::VRInitError_None;
	HMD_ = vr::VR_Init( &eError, vr::VRApplication_Scene );
	
	if (eError != vr::VRInitError_None)
	{
		HMD_ = nullptr;
		LOG(ERROR) << "Unable to init VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError);
		return false;
	}

	return true;
}

bool ftl::gui::Screen::isVR() {
	auto *cam = activeCamera();
	if (HMD_ == nullptr || cam == nullptr) { return false; }
	return cam->isVR();
}

bool ftl::gui::Screen::switchVR(bool on) {
	if (isVR() == on) { return on; }

	if (on && (HMD_ == nullptr) && !initVR()) {
		return false;
	}

	if (on) {
		activeCamera()->setVR(true);
	} else {
		activeCamera()->setVR(false);
	}
	
	return isVR();
}

bool ftl::gui::Screen::isHmdPresent() {
	return vr::VR_IsHmdPresent();
}

#endif

ftl::gui::Screen::~Screen() {
	mShader.free();

	#ifdef HAVE_OPENVR
	if (HMD_ != nullptr) {
		vr::VR_Shutdown();
	}
	#endif
}

void ftl::gui::Screen::setActiveCamera(ftl::gui::Camera *cam) {
	if (camera_) camera_->active(false);
	camera_ = cam;

	if (cam) {
		status_ = cam->name();
		mwindow_->setVisible(true);
		mwindow_->cameraChanged();
		swindow_->setVisible(false);
		cam->active(true);
	} else {
		mwindow_->setVisible(false);
		swindow_->setVisible(true);
		status_ = "[No camera]";
	}
}

bool ftl::gui::Screen::scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel) {
	if (nanogui::Screen::scrollEvent(p, rel)) {
		return true;
	} else {
		zoom_ += zoom_ * 0.1f * rel[1];
		return true;
	}
}

bool ftl::gui::Screen::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) {
	if (nanogui::Screen::mouseMotionEvent(p, rel, button, modifiers)) {
		return true;
	} else {
		if (camera_) {
			if (button == 1) {
				camera_->mouseMovement(rel[0], rel[1], button);
			} else if (button == 2) {
				pos_x_ += rel[0];
				pos_y_ += -rel[1];
			}
		}
	}
	return true;
}

bool ftl::gui::Screen::mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) {
	if (nanogui::Screen::mouseButtonEvent(p, button, down, modifiers)) {
		return true;
	} else {
		if (!camera_) return false;
		
		Eigen::Vector2f screenSize = size().cast<float>();
		auto mScale = (screenSize.cwiseQuotient(imageSize).minCoeff());
		Eigen::Vector2f scaleFactor = mScale * imageSize.cwiseQuotient(screenSize);
		Eigen::Vector2f positionInScreen(0.0f, 0.0f);
		auto mOffset = (screenSize - (screenSize.cwiseProduct(scaleFactor))) / 2;
		Eigen::Vector2f positionAfterOffset = positionInScreen + mOffset;

		float sx = ((float)p[0] - positionAfterOffset[0]) / mScale;
		float sy = ((float)p[1] - positionAfterOffset[1]) / mScale;

		if (button == 1 && down) {
			
			auto p = camera_->getPoint(sx, sy);
			points_.push_back(p);

			//auto n = camera_->getNormal(sx, sy);

			LOG(INFO) << "point: (" << p.x << ", " << p.y << ", " << p.z << ") added";
			if (points_.size() < 2) { return true; }
			
			auto p1 = Eigen::Vector3f(points_[0].x, points_[0].y, points_[0].z);
			auto p2 = Eigen::Vector3f(points_[1].x, points_[1].y, points_[1].z);
			
			points_.clear();
			// TODO: check p1 and p2 valid
			if (p1 == p2) { return true; }
			auto T = nanogui::lookAt(p1, p2, Eigen::Vector3f(0.0,1.0,0.0));
			cv::Mat T_cv;
			cv::eigen2cv(T, T_cv);
			T_cv.convertTo(T_cv, CV_64FC1);
			net_->broadcast("set_pose_adjustment", T_cv);
			
			return true;

		}
		else if (button == 0 && down) {
			auto p = camera_->getPoint(sx, sy);
			LOG(INFO) << "point: " << (Eigen::Vector4d(p.x, p.y, p.z, -1.0f)).transpose();

			//auto q = camera_->getNormal(sx, sy);
			//LOG(INFO) << "normal: " << (Eigen::Vector4d(q.x, q.y, q.z, 1.0)).transpose();
		}
		return false;
	}
}

static std::string generateKeyComboStr(int key, int modifiers) {
	std::string res = "";

	switch(modifiers) {
	case 1:		res += "Shift+"; break;
	case 2:		res += "Ctrl+"; break;
	case 3:		res += "Ctrl+Shift+"; break;
	case 4:		res += "Alt+"; break;
	default: break;
	}

	if (key < 127 && key >= 32) {
		char buf[2] = { (char)key, 0 };
		return res + std::string(buf);
	} else {
		return "";
	}
}

bool ftl::gui::Screen::keyboardEvent(int key, int scancode, int action, int modifiers) {
	using namespace Eigen;
	if (nanogui::Screen::keyboardEvent(key, scancode, action, modifiers)) {
		return true;
	} else {
		//LOG(INFO) << "Key press " << key << " - " << action << " - " << modifiers;

		if ((key >= 262 && key <= 267) || (key >= '0' && key <= '9')) {
			if (camera_) camera_->keyMovement(key, modifiers);
			return true;
		} else if (action == 1 && key == 'H') {
			swindow_->setVisible(false);
			//cwindow_->setVisible(false);
		} else if (action == 1) {
			std::string combo = generateKeyComboStr(key, modifiers);

			if (combo.size() > 0) {
				LOG(INFO) << "Key combo = " << combo;

				auto s = shortcuts_->get<nlohmann::json>(combo);
				if (s) {
					//LOG(INFO) << "FOUND KEYBOARD SHORTCUT";
					std::string op = (*s).value("op",std::string("="));
					std::string uri = (*s).value("uri",std::string(""));

					if (op == "toggle") {
						auto v = ftl::config::get(uri);
						if (v.is_boolean()) {
							ftl::config::update(uri, !v.get<bool>());
						}
					} else if (op == "+=") {
						auto v = ftl::config::get(uri);
						if (v.is_number_float()) {
							ftl::config::update(uri, v.get<float>() + (*s).value("value",0.0f));
						} else if (v.is_number_integer()) {
							ftl::config::update(uri, v.get<int>() + (*s).value("value",0));
						}
					} else if (op == "-=") {
						auto v = ftl::config::get(uri);
						if (v.is_number_float()) {
							ftl::config::update(uri, v.get<float>() - (*s).value("value",0.0f));
						} else if (v.is_number_integer()) {
							ftl::config::update(uri, v.get<int>() - (*s).value("value",0));
						}
					} else if (op == "=") {
						ftl::config::update(uri, (*s)["value"]);
					}
				}
			}
		}
		return false;
	}
}

void ftl::gui::Screen::draw(NVGcontext *ctx) {
	using namespace Eigen;

	Vector2f screenSize = size().cast<float>();

	if (camera_) {
		imageSize = {camera_->width(), camera_->height()};

		//if (camera_->getChannel() != ftl::codecs::Channel::Left) { mImageID = rightEye_; }

		if (camera_->getLeft().isValid() && imageSize[0] > 0) {
			auto mScale = (screenSize.cwiseQuotient(imageSize).minCoeff()) * zoom_;
			Vector2f scaleFactor = mScale * imageSize.cwiseQuotient(screenSize);
			Vector2f positionInScreen(pos_x_, pos_y_);
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
			glBindTexture(GL_TEXTURE_2D, leftEye_);
			//camera_->getLeft().texture();
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, (camera_->isStereo() && camera_->getRight().isValid()) ? rightEye_ : leftEye_);
			glActiveTexture(GL_TEXTURE2);
			glBindTexture(GL_TEXTURE_2D, camera_->getDepth().texture());
			//(camera_->isStereo() && camera_->getRight().isValid()) ? camera_->getRight().texture() : camera_->getLeft().texture();
			mShader.setUniform("image1", 0);
			mShader.setUniform("image2", 1);
			mShader.setUniform("depthImage", 2);
			mShader.setUniform("blendAmount", (camera_->isStereo()) ? root_->value("blending", 0.5f) : 1.0f);
			mShader.setUniform("scaleFactor", scaleFactor);
			mShader.setUniform("position", imagePosition);

			glEnable(GL_DEPTH_TEST); 
			glDepthMask(GL_TRUE);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			mShader.drawIndexed(GL_TRIANGLES, 0, 2);
			//glDisable(GL_SCISSOR_TEST);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

			camera_->drawOverlay(screenSize);
			 
			glDisable(GL_DEPTH_TEST);
		}
	} else {
		// Must periodically render the cameras here to update any thumbnails.
		auto cams = swindow_->getCameras();
		for (auto *c : cams) {
			c->drawUpdated(swindow_->getFramesets());
		}
	}

	nvgTextAlign(ctx, NVG_ALIGN_RIGHT);

	if (root()->value("show_information", true)) {
		string msg;

		auto &stats = getStatistics();

		msg = string("Frame rate: ") + std::to_string((int)stats.fps);
		nvgText(ctx, screenSize[0]-10, 20, msg.c_str(), NULL);
		msg = string("Latency: ") + std::to_string((int)stats.latency) + string("ms");
		nvgText(ctx, screenSize[0]-10, 40, msg.c_str(), NULL);	

		msg = string("Bitrate: ") + to_string_with_precision(stats.bitrate, 2) + string("Mbps");
		nvgText(ctx, screenSize[0]-10, 60, msg.c_str(), NULL);

		if (camera_) {
			auto intrin = camera_->getIntrinsics();
			msg = string("Resolution: ") + std::to_string(intrin.width) + string("x") + std::to_string(intrin.height);
			nvgText(ctx, screenSize[0]-10, 80, msg.c_str(), NULL);
			msg = string("Focal: ") + to_string_with_precision(intrin.fx, 2);
			nvgText(ctx, screenSize[0]-10, 100, msg.c_str(), NULL);
		}
	}

	nvgText(ctx, screenSize[0]-10, screenSize[1]-20, status_.c_str(), NULL);

	/* Draw the user interface */
	screen()->performLayout(ctx);
	nanogui::Screen::draw(ctx);
}

const ftl::gui::Statistics &ftl::gui::Screen::getStatistics() {
	if (--last_stats_count_ <= 0) {
		auto [fps,latency] = ftl::rgbd::Builder::getStatistics();
		stats_.fps = fps;
		stats_.latency = latency;
		stats_.bitrate = ftl::stream::Net::getRequiredBitrate();
		last_stats_count_ = 20;
	}
	return stats_;
}

void ftl::gui::Screen::drawFast() {
	if (camera_) {
		camera_->captureFrame();

		glActiveTexture(GL_TEXTURE0);
		mImageID = camera_->getLeft().texture();
		leftEye_ = mImageID;
		rightEye_ = camera_->getRight().texture();

		#ifdef HAVE_OPENVR
		if (isVR() && camera_->width() > 0 && camera_->getLeft().isValid() && camera_->getRight().isValid()) {
			
			//glBindTexture(GL_TEXTURE_2D, leftEye_);
			vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)leftEye_, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
			vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );

			//glBindTexture(GL_TEXTURE_2D, rightEye_);
			vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)rightEye_, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
			vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );

			glFlush();
		}
		#endif
	}
}
