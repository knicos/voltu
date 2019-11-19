#include "config_window.hpp"

#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/entypo.h>
#include <nanogui/formhelper.h>
#include <nanogui/vscrollpanel.h>

#include <vector>
#include <string>

using ftl::gui::ConfigWindow;
using std::string;
using std::vector;
using ftl::config::json_t;

ConfigWindow::ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl)
		: nanogui::Window(parent, "Settings"), ctrl_(ctrl) {
	using namespace nanogui;

	setLayout(new GroupLayout());
	setPosition(Vector2i(parent->width()/2.0f - 100.0f, parent->height()/2.0f - 100.0f));
	//setModal(true);

	configurables_ = ftl::config::list();

	new Label(this, "Select Configurable","sans-bold");

	auto vscroll = new VScrollPanel(this);
	vscroll->setFixedHeight(300);
	Widget *buttons = new Widget(vscroll);
	buttons->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill));

	for (auto c : configurables_) {
		auto itembutton = new Button(buttons, c);
		itembutton->setCallback([this,c]() {
			LOG(INFO) << "Change configurable: " << c;
			_buildForm(c);
			setVisible(false);
			//this->parent()->removeChild(this);
			//delete this;
			//screen()->removeChild(this);
		});
	}
}

ConfigWindow::~ConfigWindow() {

}

void ConfigWindow::_addElements(nanogui::FormHelper *form, const std::string &suri) {
	using namespace nanogui;

	Configurable *configurable = ftl::config::find(suri);
	ftl::config::json_t data;
	if (configurable) {
		data = configurable->getConfig();
	}

	for (auto i=data.begin(); i!=data.end(); ++i) {
		if (i.key() == "$id") continue;

		if (i.key() == "$ref" && i.value().is_string()) {
			LOG(INFO) << "Follow $ref: " << i.value();
			const std::string suri = std::string(i.value().get<string>());
			_addElements(form, suri);
			continue;
		}

		if (i.value().is_boolean()) {
			string key = i.key();
			form->addVariable<bool>(i.key(), [this,data,key,suri](const bool &b){
				ftl::config::update(suri+"/"+key, b);
			}, [data,key]() -> bool {
				return data[key].get<bool>();
			});
		} else if (i.value().is_number_integer()) {
			string key = i.key();
			form->addVariable<int>(i.key(), [this,data,key,suri](const int &f){
				ftl::config::update(suri+"/"+key, f);
			}, [data,key]() -> int {
				return data[key].get<int>();
			});
		} else if (i.value().is_number_float()) {
			string key = i.key();
			form->addVariable<float>(i.key(), [this,data,key,suri](const float &f){
				ftl::config::update(suri+"/"+key, f);
			}, [data,key]() -> float {
				return data[key].get<float>();
			});
		} else if (i.value().is_string()) {
			string key = i.key();
			form->addVariable<string>(i.key(), [this,data,key,suri](const string &f){
				ftl::config::update(suri+"/"+key, f);
			}, [data,key]() -> string {
				return data[key].get<string>();
			});
		} else if (i.value().is_object()) {
			string key = i.key();
		
			// Checking the URI with exists() prevents unloaded local configurations from being shown.
			if (suri.find('#') != string::npos && exists(suri+string("/")+key)) {
				form->addButton(key, [this,suri,key]() {
					_buildForm(suri+string("/")+key);
				})->setIcon(ENTYPO_ICON_FOLDER);
			} else if (exists(suri+string("#")+key)) {
				form->addButton(key, [this,suri,key]() {
					_buildForm(suri+string("#")+key);
				})->setIcon(ENTYPO_ICON_FOLDER);
			}
		}
	}
}

void ConfigWindow::_buildForm(const std::string &suri) {
	using namespace nanogui;

	ftl::URI uri(suri);

	FormHelper *form = new FormHelper(this->screen());
	//form->setWindow(this);
	form->addWindow(Vector2i(100,50), uri.getFragment());
	form->window()->setTheme(theme());

	_addElements(form, suri);

	auto closebutton = form->addButton("Close", [this,form]() {
		form->window()->setVisible(false);
		delete form;
	});
	closebutton->setIcon(ENTYPO_ICON_CROSS);
}

bool ConfigWindow::exists(const std::string &uri) {
	return ftl::config::find(uri) != nullptr;
}
