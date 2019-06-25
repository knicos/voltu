#include "config_window.hpp"

#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/combobox.h>
#include <nanogui/button.h>
#include <nanogui/entypo.h>
#include <nanogui/formhelper.h>

#include <vector>
#include <string>

using ftl::gui::ConfigWindow;
using std::string;
using std::vector;
using ftl::config::json_t;


ConfigWindow::ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl, const ftl::UUID &peer)
		: nanogui::Window(parent, "Settings"), ctrl_(ctrl), peer_(peer) {
	using namespace nanogui;

	setLayout(new GroupLayout());
	setPosition(Vector2i(parent->width()/2.0f - 100.0f, parent->height()/2.0f - 100.0f));
	//setModal(true);

	configurables_ = ctrl->getConfigurables(peer);

	auto label = new Label(this, "Select Configurable","sans-bold");

	auto select = new ComboBox(this, configurables_);
	select->setCallback([this](int ix) {
		LOG(INFO) << "Change configurable: " << ix;
		_buildForm(configurables_[ix], ctrl_->get(peer_, configurables_[ix]));

		setVisible(false);
		//this->parent()->removeChild(this);
		//delete this;
		//screen()->removeChild(this);
	});
}

ConfigWindow::~ConfigWindow() {

}

void ConfigWindow::_addElements(nanogui::FormHelper *form, const std::string &suri, const ftl::config::json_t &data) {
	using namespace nanogui;

	for (auto i=data.begin(); i!=data.end(); ++i) {
		if (i.key() == "$id") continue;

		if (i.key() == "$ref" && i.value().is_string()) {
			LOG(INFO) << "Follow $ref: " << i.value();
			_addElements(form, suri, ctrl_->get(peer_, i.value().get<string>()));
			continue;
		}

		if (i.value().is_boolean()) {
			string key = i.key();
			form->addVariable<bool>(i.key(), [this,data,key,suri](const bool &b){
				ctrl_->set(peer_, suri + string("/") + key, json_t(b));
			}, [data,key]() -> bool {
				return data[key].get<bool>();
			});
		} else if (i.value().is_number_integer()) {
			string key = i.key();
			form->addVariable<int>(i.key(), [this,data,key,suri](const int &f){
				ctrl_->set(peer_, suri + string("/") + key, json_t(f));
			}, [data,key]() -> int {
				return data[key].get<int>();
			});
		} else if (i.value().is_number_float()) {
			string key = i.key();
			form->addVariable<float>(i.key(), [this,data,key,suri](const float &f){
				ctrl_->set(peer_, suri + string("/") + key, json_t(f));
			}, [data,key]() -> float {
				return data[key].get<float>();
			});
		} else if (i.value().is_string()) {
			string key = i.key();
			form->addVariable<string>(i.key(), [this,data,key,suri](const string &f){
				ctrl_->set(peer_, suri + string("/") + key, json_t(f));
			}, [data,key]() -> string {
				return data[key].get<string>();
			});
		} else if (i.value().is_object()) {
			string key = i.key();
			const ftl::config::json_t &v = i.value();
		
			form->addButton(i.key(), [this,form,suri,key,v]() {
				_buildForm(suri+string("/")+key, v);
			})->setIcon(ENTYPO_ICON_FOLDER);
		}
	}
}

void ConfigWindow::_buildForm(const std::string &suri, ftl::config::json_t data) {
	using namespace nanogui;

	ftl::URI uri(suri);

	FormHelper *form = new FormHelper(this->screen());
	//form->setWindow(this);
	form->addWindow(Vector2i(100,50), uri.getFragment());
	form->window()->setTheme(theme());

	_addElements(form, suri, data);

	auto closebutton = form->addButton("Close", [form]() {
		form->window()->setVisible(false);
		delete form;
	});
	closebutton->setIcon(ENTYPO_ICON_CROSS);
}
