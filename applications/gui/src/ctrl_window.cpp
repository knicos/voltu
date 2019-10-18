#include "ctrl_window.hpp"

#include "config_window.hpp"

#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/combobox.h>
#include <nanogui/button.h>
#include <nanogui/entypo.h>

#include <vector>
#include <string>

using ftl::gui::ControlWindow;
using ftl::gui::ConfigWindow;
using std::string;
using std::vector;


ControlWindow::ControlWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl)
		: nanogui::Window(parent, "Network Connections"), ctrl_(ctrl) {
	setLayout(new nanogui::GroupLayout());

	using namespace nanogui;

	_updateDetails();

	auto tools = new Widget(this);
	tools->setLayout(new BoxLayout(	Orientation::Horizontal,
									Alignment::Middle, 0, 6));

	auto button = new Button(tools, "", ENTYPO_ICON_PLUS);
	button->setCallback([this] {
		// Show new connection dialog
		_addNode();
	});
	button->setTooltip("Add new node");
	
	// commented-out buttons not working/useful
	/*
	button = new Button(tools, "", ENTYPO_ICON_CYCLE);
	button->setCallback([this] {
		ctrl_->restart();
	});
	button = new Button(tools, "", ENTYPO_ICON_CONTROLLER_PAUS);
	button->setCallback([this] {
		ctrl_->pause();
	});
	button->setTooltip("Pause all nodes");*/

	new Label(this, "Select Node","sans-bold");
	auto select = new ComboBox(this, node_titles_);
	select->setCallback([this](int ix) {
		LOG(INFO) << "Change node: " << ix;
		_changeActive(ix);
	});

	new Label(this, "Node Options","sans-bold");

	tools = new Widget(this);
	tools->setLayout(new BoxLayout(	Orientation::Horizontal,
									Alignment::Middle, 0, 6));

	/*button = new Button(tools, "", ENTYPO_ICON_INFO);
	button->setCallback([this] {
		
	});
	button->setTooltip("Node status information");*/

	button = new Button(tools, "", ENTYPO_ICON_COG);
	button->setCallback([this,parent] {
		auto cfgwin = new ConfigWindow(parent, ctrl_, _getActiveID());
		cfgwin->setTheme(theme());
	});
	button->setTooltip("Edit node configuration");

	/*button = new Button(tools, "", ENTYPO_ICON_CYCLE);
	button->setCallback([this] {
		ctrl_->restart(_getActiveID());
	});
	button->setTooltip("Restart this node");*/

	/*button = new Button(tools, "", ENTYPO_ICON_CONTROLLER_PAUS);
	button->setCallback([this] {
		ctrl_->pause(_getActiveID());
	});
	button->setTooltip("Pause node processing");*/

	ctrl->getNet()->onConnect([this,select](ftl::net::Peer *p) {
		_updateDetails();
		select->setItems(node_titles_);
	});

	_changeActive(0);
}

ControlWindow::~ControlWindow() {

}

void ControlWindow::_addNode() {
	using namespace nanogui;

	FormHelper *form = new FormHelper(this->screen());
	form->addWindow(Vector2i(100,100), "Add Node");

	auto var = form->addVariable("URI", add_node_uri_);
	var->setValue("tcp://localhost:9001");
	var->setFixedWidth(200);

	form->addButton("Add", [this,form](){
		ctrl_->getNet()->connect(add_node_uri_);
		form->window()->setVisible(false);
		delete form;
	})->setIcon(ENTYPO_ICON_PLUS);

	form->addButton("Close", [form]() {
		form->window()->setVisible(false);
		delete form;
	})->setIcon(ENTYPO_ICON_CROSS);
}

void ControlWindow::_updateDetails() {
	node_details_ = ctrl_->getSlaves();

	node_titles_.clear();
	for (auto &d : node_details_) {
		node_titles_.push_back(d["title"].get<string>());
	}
}

void ControlWindow::_changeActive(int ix) {
	active_ix_ = ix;
}

ftl::UUID ControlWindow::_getActiveID() {
	return ftl::UUID(node_details_[active_ix_]["id"].get<string>());
}

