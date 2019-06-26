#include "pose_window.hpp"

#include <nanogui/combobox.h>
#include <nanogui/label.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>

using ftl::gui::PoseWindow;
using std::string;

PoseWindow::PoseWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl, const std::string &src)
		: nanogui::Window(parent, "Pose Adjust"), ctrl_(ctrl), src_(src) {
	using namespace nanogui;

	//setLayout(new nanogui::GroupLayout());
	setLayout(new BoxLayout(Orientation::Vertical,
                                       Alignment::Middle, 0, 6));
	
	pose_param_ = kPoseTranslation;
	pose_precision_ = 0.1;

	pose_ = ctrl_->getPose(src_);

	//Widget *tools = new Widget(this);
	//    tools->setLayout(new BoxLayout(Orientation::Horizontal,
	//                                   Alignment::Middle, 0, 6));

	auto grouping = new Widget(this);
	grouping->setLayout(new GroupLayout());

	new Label(grouping, "Select source","sans-bold");
	available_ = ctrl->getNet()->findAll<string>("list_streams");
	auto select = new ComboBox(grouping, available_);
	select->setSelectedIndex(std::distance(available_.begin(), std::find(available_.begin(), available_.end(), src_)));
	select->setCallback([this,select](int ix) {
		src_ = available_[ix];
		pose_ = ctrl_->getPose(src_);
	});

	ctrl->getNet()->onConnect([this,select](ftl::net::Peer *p) {
		available_ = ctrl_->getNet()->findAll<string>("list_streams");
		select->setItems(available_);
	});

	new Label(grouping, "Pose Options","sans-bold");

	auto tools = new Widget(grouping);
    tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                       Alignment::Middle, 0, 6));

	auto button_rgb = new Button(tools, "Translation");
	button_rgb->setTooltip("Adjust camera location");
	button_rgb->setFlags(Button::RadioButton);
	button_rgb->setPushed(true);
	button_rgb->setChangeCallback([this](bool state) { pose_param_ = kPoseTranslation; });

	auto button_depth = new Button(tools, "Rotation");
	button_depth->setFlags(Button::RadioButton);
	button_depth->setChangeCallback([this](bool state) { pose_param_ = kPoseRotation; });

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
	

	new Widget(tools);
	button = new Button(tools, "", ENTYPO_ICON_CHEVRON_UP);
	button->setCallback([this]() {
		Eigen::Affine3d transform(Eigen::Translation3d(0.0,0.0,-pose_precision_));
		Eigen::Matrix4d matrix = transform.matrix();
		pose_ *= matrix;
		ctrl_->setPose(src_, pose_);
	});
	new Widget(tools);

	button = new Button(tools, "", ENTYPO_ICON_CHEVRON_LEFT);
	button->setCallback([this]() {
		Eigen::Affine3d transform(Eigen::Translation3d(-pose_precision_,0.0,0.0));
		Eigen::Matrix4d matrix = transform.matrix();
		pose_ *= matrix;
		ctrl_->setPose(src_, pose_);
	});
	new Widget(tools);
	button = new Button(tools, "", ENTYPO_ICON_CHEVRON_RIGHT);
	button->setCallback([this]() {
		Eigen::Affine3d transform(Eigen::Translation3d(pose_precision_,0.0,0.0));
		Eigen::Matrix4d matrix = transform.matrix();
		pose_ *= matrix;
		ctrl_->setPose(src_, pose_);
	});

	new Widget(tools);
	button = new Button(tools, "", ENTYPO_ICON_CHEVRON_DOWN);
	button->setCallback([this]() {
		Eigen::Affine3d transform(Eigen::Translation3d(0.0,0.0,pose_precision_));
		Eigen::Matrix4d matrix = transform.matrix();
		pose_ *= matrix;
		ctrl_->setPose(src_, pose_);
	});
}

PoseWindow::~PoseWindow() {

}
