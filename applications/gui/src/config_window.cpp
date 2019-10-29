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

ConfigWindow::ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl, const ftl::UUID &peer) : ConfigWindow(parent, ctrl, std::optional<ftl::UUID>(peer)) {

}

ConfigWindow::ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl, const std::optional<ftl::UUID> &peer)
		: nanogui::Window(parent, "Settings"), ctrl_(ctrl), peer_(peer) {
	using namespace nanogui;

	setLayout(new GroupLayout());
	setPosition(Vector2i(parent->width()/2.0f - 100.0f, parent->height()/2.0f - 100.0f));
	//setModal(true);

	if (peer) {
		configurables_ = ctrl->getConfigurables(peer.value());
	} else {
		configurables_ = ftl::config::list();
	}

	new Label(this, "Select Configurable","sans-bold");

	for (auto c : configurables_) {
		auto itembutton = new Button(this, c);
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

class ConfigWindow::References {
	public:
	References(ftl::NetConfigurable* nc, ftl::config::json_t* config, const std::string* suri) : nc(nc), config(config), suri(suri) {
	}

	~References() {
		delete nc;
		delete config;
		delete suri;
	}

	private:
	ftl::NetConfigurable* nc;
	ftl::config::json_t* config;
	const std::string* suri;
};

std::vector<ftl::gui::ConfigWindow::References *> ConfigWindow::_addElements(nanogui::FormHelper *form, ftl::Configurable &nc, const std::string &suri, std::function<ftl::Configurable*(const std::string*, std::vector<References *>&)> construct) {
	using namespace nanogui;

	std::vector<References *> references;

	auto data = nc.getConfig();

	for (auto i=data.begin(); i!=data.end(); ++i) {
		if (i.key() == "$id") continue;

		if (i.key() == "$ref" && i.value().is_string()) {
			LOG(INFO) << "Follow $ref: " << i.value();
			const std::string* suri = new std::string(i.value().get<string>());
			ftl::Configurable* rc = construct(suri, references);
			auto new_references = _addElements(form, *rc, *suri, construct);
			references.insert(references.end(), new_references.begin(), new_references.end());
			continue;
		}

		if (i.value().is_boolean()) {
			string key = i.key();
			form->addVariable<bool>(i.key(), [this,data,key,&nc](const bool &b){
				nc.set(key, b);
			}, [data,key]() -> bool {
				return data[key].get<bool>();
			});
		} else if (i.value().is_number_integer()) {
			string key = i.key();
			form->addVariable<int>(i.key(), [this,data,key,&nc](const int &f){
				nc.set(key, f);
			}, [data,key]() -> int {
				return data[key].get<int>();
			});
		} else if (i.value().is_number_float()) {
			string key = i.key();
			form->addVariable<float>(i.key(), [this,data,key,&nc](const float &f){
				nc.set(key, f);
			}, [data,key]() -> float {
				return data[key].get<float>();
			});
		} else if (i.value().is_string()) {
			string key = i.key();
			form->addVariable<string>(i.key(), [this,data,key,&nc](const string &f){
				nc.set(key, f);
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

	return references;
}

void ConfigWindow::_buildForm(const std::string &suri) {
	using namespace nanogui;

	ftl::URI uri(suri);

	FormHelper *form = new FormHelper(this->screen());
	//form->setWindow(this);
	form->addWindow(Vector2i(100,50), uri.getFragment());
	form->window()->setTheme(theme());

	ftl::config::json_t* config;
	config = new ftl::config::json_t;
	const std::string* allocated_suri = new std::string(suri);
	std::vector<ftl::gui::ConfigWindow::References *> references;

	ftl::Configurable* nc;

	if (peer_) {
		*config = ctrl_->get(peer_.value(), suri);
		nc = new ftl::NetConfigurable(peer_.value(), *allocated_suri, *ctrl_, *config);

		references = _addElements(form, *nc, *allocated_suri, [this](auto suri, auto &references) {
			ftl::config::json_t* config = new ftl::config::json_t;
			*config = ctrl_->get(peer_.value(), *suri);
			auto nc = new ftl::NetConfigurable(peer_.value(), *suri, *ctrl_, *config);
			auto r = new References(nc, config, suri);
			references.push_back(r);
			return nc;
		});
	} else {
		nc = ftl::config::find(suri);
		if (nc) {
			references = _addElements(form, *nc, *allocated_suri, [this](auto suri, auto &references) {
				return ftl::config::find(*suri);
			});
		}
	}

	auto closebutton = form->addButton("Close", [this,form,config,allocated_suri,nc,references]() {
		form->window()->setVisible(false);
		for(auto r : references) {
			delete r;
		}
		if (peer_) {
			delete nc;
		}
		delete config;
		delete allocated_suri;
		delete form;
	});
	closebutton->setIcon(ENTYPO_ICON_CROSS);
}

bool ConfigWindow::exists(const std::string &uri) {
	// If the Configurable is a NetConfigurable, the URI is not checked.
	return peer_ || ftl::config::find(uri);
}