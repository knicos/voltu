#include "pose_window.hpp"
#include "screen.hpp"
#include "camera.hpp"

#include <nanogui/combobox.h>
#include <nanogui/label.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>

using ftl::gui::PoseWindow;
using ftl::gui::Screen;
using std::string;

static Eigen::Affine3d create_rotation_matrix(float ax, float ay, float az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return ry * rz * rx;
}

PoseWindow::PoseWindow(ftl::gui::Screen *screen, const std::string &src)
		: nanogui::Window(screen, "Pose Adjust"), src_(src), screen_(screen) {
	using namespace nanogui;

	//setLayout(new nanogui::GroupLayout());
	setLayout(new BoxLayout(Orientation::Vertical,
                                       Alignment::Middle, 0, 6));
	
	pose_param_ = kPoseTranslation;
	pose_precision_ = 0.1;

	pose_ = screen_->control()->getPose(src_);

	//Widget *tools = new Widget(this);
	//    tools->setLayout(new BoxLayout(Orientation::Horizontal,
	//                                   Alignment::Middle, 0, 6));

	auto grouping = new Widget(this);
	grouping->setLayout(new GroupLayout());

	new Label(grouping, "Select source","sans-bold");
	available_ = screen_->net()->findAll<string>("list_streams");
	auto select = new ComboBox(grouping, available_);
	select->setSelectedIndex(std::distance(available_.begin(), std::find(available_.begin(), available_.end(), src_)));
	select->setCallback([this,select](int ix) {
		src_ = available_[ix];
		pose_ = screen_->control()->getPose(src_);
	});

	screen_->net()->onConnect([this,select](ftl::net::Peer *p) {
		available_ = screen_->control()->getNet()->findAll<string>("list_streams");
		select->setItems(available_);
	});

	new Label(grouping, "Pose Options","sans-bold");

	auto tools = new Widget(grouping);
    tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                       Alignment::Middle, 0, 6));

	auto button_opt = new Button(tools, "", ENTYPO_ICON_EYE);
	button_opt->setTooltip("Virtual view to this pose");
	//button_opt->setFlags(Button::ToggleButton);
	//button_opt->setPushed(false);
	button_opt->setCallback([this]() {
		screen_->activeCamera()->setPose(pose_);
	});

	button_opt = new Button(tools, "", ENTYPO_ICON_LINK);
	button_opt->setTooltip("Link virtual current pose to this pose");
	button_opt->setFlags(Button::ToggleButton);
	button_opt->setPushed(false);
	button_opt->setChangeCallback([this](bool state) {  });

	tools = new Widget(grouping);
    tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                       Alignment::Middle, 0, 6));

	auto button_rgb = new Button(tools, "Translation");
	button_rgb->setTooltip("Adjust camera location");
	button_rgb->setFlags(Button::RadioButton);
	button_rgb->setPushed(true);
	button_rgb->setCallback([this]() { pose_param_ = kPoseTranslation; });

	auto button_depth = new Button(tools, "Rotation");
	button_depth->setFlags(Button::RadioButton);
	button_depth->setCallback([this]() { pose_param_ = kPoseRotation; });

	auto button_stddev = new Button(tools, "Raw");
	button_stddev->setTooltip("Edit the numbers directly");
	button_stddev->setFlags(Button::RadioButton);
	button_stddev->setChangeCallback([this](bool state) { pose_param_ = kPoseRaw; });

	tools = new Widget(grouping);
    tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                       Alignment::Middle, 0, 6));

	auto button = new Button(tools, "0.1m");
	button->setFlags(Button::RadioButton);
	button->setPushed(true);
	button->setCallback([this](){
		pose_precision_ = 0.1f;
	});

	button = new Button(tools, "0.01m");
	button->setFlags(Button::RadioButton);
	button->setCallback([this](){
		pose_precision_ = 0.01f;
	});

	button = new Button(tools, "0.001m");
	button->setFlags(Button::RadioButton);
	button->setCallback([this](){
		pose_precision_ = 0.001f;
	});

	tools = new Widget(this);
	auto grid = new GridLayout(Orientation::Horizontal, 3, Alignment::Middle, 5, 4);
	tools->setLayout(grid);
	tools->setFixedWidth(150);
	

	button = new Button(tools, "Up");
	button->setCallback([this]() {
		if (pose_param_ ==  kPoseTranslation) {
			Eigen::Affine3d transform(Eigen::Translation3d(0.0,-pose_precision_,0.0));
			Eigen::Matrix4d matrix = transform.matrix();
			pose_ *= matrix;
		} else if (pose_param_ == kPoseRotation) {
			Eigen::Affine3d r = create_rotation_matrix(pose_precision_, 0.0, 0.0);
			pose_ = r.matrix() * pose_;
		}
		
		screen_->control()->setPose(src_, pose_);
	});
	button = new Button(tools, "", ENTYPO_ICON_CHEVRON_UP);
	button->setCallback([this]() {
		if (pose_param_ == kPoseTranslation) {
			Eigen::Affine3d transform(Eigen::Translation3d(0.0,0.0,-pose_precision_));
			Eigen::Matrix4d matrix = transform.matrix();
			pose_ *= matrix;
		} else if (pose_param_ == kPoseRotation) {
			Eigen::Affine3d r = create_rotation_matrix(0.0, 0.0, pose_precision_);
			pose_ = r.matrix() * pose_;
		}
		screen_->control()->setPose(src_, pose_);
	});
	button = new Button(tools, "Down");
	button->setCallback([this]() {
		if (pose_param_ == kPoseTranslation) {
			Eigen::Affine3d transform(Eigen::Translation3d(0.0,pose_precision_,0.0));
			Eigen::Matrix4d matrix = transform.matrix();
			pose_ *= matrix;
		} else if (pose_param_ == kPoseRotation) {
			Eigen::Affine3d r = create_rotation_matrix(-pose_precision_, 0.0, 0.0);
			pose_ = r.matrix() * pose_;
		}
		screen_->control()->setPose(src_, pose_);
	});

	button = new Button(tools, "", ENTYPO_ICON_CHEVRON_LEFT);
	button->setCallback([this]() {
		if (pose_param_ == kPoseTranslation) {
			Eigen::Affine3d transform(Eigen::Translation3d(-pose_precision_,0.0,0.0));
			Eigen::Matrix4d matrix = transform.matrix();
			pose_ *= matrix;
		} else if (pose_param_ == kPoseRotation) {
			Eigen::Affine3d r = create_rotation_matrix(0.0, pose_precision_, 0.0);
			pose_ = r.matrix() * pose_;
		}
		screen_->control()->setPose(src_, pose_);
	});
	new Widget(tools);
	button = new Button(tools, "", ENTYPO_ICON_CHEVRON_RIGHT);
	button->setCallback([this]() {
		if (pose_param_ == kPoseTranslation) {
			Eigen::Affine3d transform(Eigen::Translation3d(pose_precision_,0.0,0.0));
			Eigen::Matrix4d matrix = transform.matrix();
			pose_ *= matrix;
		} else if (pose_param_ == kPoseRotation) {
			Eigen::Affine3d r = create_rotation_matrix(0.0, -pose_precision_, 0.0);
			pose_ = r.matrix() * pose_;
		}
		screen_->control()->setPose(src_, pose_);
	});

	new Widget(tools);
	button = new Button(tools, "", ENTYPO_ICON_CHEVRON_DOWN);
	button->setCallback([this]() {
		if (pose_param_ == kPoseTranslation) {
			Eigen::Affine3d transform(Eigen::Translation3d(0.0,0.0,pose_precision_));
			Eigen::Matrix4d matrix = transform.matrix();
			pose_ *= matrix;
		} else if (pose_param_ == kPoseRotation) {
			Eigen::Affine3d r = create_rotation_matrix(0.0, 0.0, -pose_precision_);
			pose_ = r.matrix() * pose_;
		}
		screen_->control()->setPose(src_, pose_);
	});
}

PoseWindow::~PoseWindow() {

}
