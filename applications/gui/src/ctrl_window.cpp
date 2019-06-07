#include "ctrl_window.hpp"

#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/combobox.h>
#include <nanogui/button.h>
#include <nanogui/entypo.h>

#include <vector>
#include <string>

using ftl::gui::ControlWindow;
using std::string;
using std::vector;


ControlWindow::ControlWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl)
		: nanogui::Window(parent, "Node Control"), ctrl_(ctrl) {
	setLayout(new nanogui::GroupLayout());

	using namespace nanogui;

	_updateDetails();

	auto button = new Button(this, "Add Node", ENTYPO_ICON_PLUS);
	button->setCallback([this] {
		// Show new connection dialog
	});
	button = new Button(this, "Restart All", ENTYPO_ICON_CYCLE);
	button->setCallback([this] {
		ctrl_->restart();
	});
	button = new Button(this, "Shutdown All", ENTYPO_ICON_POWER_PLUG);
	button->setCallback([this] {
		ctrl_->shutdown();
	});

	new Label(this, "Select Node","sans-bold");
	auto select = new ComboBox(this, node_titles_);
	select->setCallback([this](int ix) {
		LOG(INFO) << "Change node: " << ix;
		_changeActive(ix);
	});

	button = new Button(this, "Restart Node", ENTYPO_ICON_CYCLE);
	button->setCallback([this] {
		ctrl_->restart(_getActiveID());
	});

	button = new Button(this, "Shutdown Node", ENTYPO_ICON_POWER_PLUG);
	button->setCallback([this] {
		ctrl_->shutdown(_getActiveID());
	});

	_changeActive(0);
}

ControlWindow::~ControlWindow() {

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

