#include "config_window.hpp"

#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/entypo.h>
#include <nanogui/formhelper.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/opengl.h>

#include <vector>
#include <string>

using ftl::gui::ConfigWindow;
using std::string;
using std::vector;
using ftl::config::json_t;

class SearchBox : public nanogui::TextBox {
private:
	std::vector<std::string> configurables_;
	Widget *buttons_;
	std::string previous;

	void _setVisible(const std::string &str) {
		// Check whether the search string has changed to prevent
		// unnecessary searching.
		if (str != previous) {
			for (int i = configurables_.size()-1; i >= 0; --i) {
				if (configurables_[i].find(mValueTemp) != std::string::npos) {
					buttons_->childAt(i)->setVisible(true);
				} else {
					buttons_->childAt(i)->setVisible(false);
				}
			}
			previous = str;
		}
	}

public:
	SearchBox(Widget *parent, std::vector<std::string> &configurables) : nanogui::TextBox(parent, ""), configurables_(configurables) {
		setAlignment(TextBox::Alignment::Left);
		setEditable(true);
		setPlaceholder("Search");
	}

	~SearchBox() {
	}

	bool keyboardEvent(int key, int scancode, int action, int modifier) {
		TextBox::keyboardEvent(key, scancode, action, modifier);
		_setVisible(mValueTemp);
		return true;
	}

	void setButtons(Widget *buttons) {
		buttons_ = buttons;
	}
};

static std::string titleForURI(const ftl::URI &uri) {
	auto *cfg = ftl::config::find(uri.getBaseURI());
	if (cfg && cfg->get<std::string>("title")) {
		return *cfg->get<std::string>("title");
	} else if (uri.getPath().size() > 0) {
		return uri.getPathSegment(-1);
	} else {
		return uri.getHost();
	}
}

static std::string genPadding(const std::string &str, size_t count) {
	std::string res = "";
	for (size_t i=0; i<count; ++i) {
		res += str;
	}
	return res;
}

ConfigWindow::ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl)
		: nanogui::Window(parent, "Settings"), ctrl_(ctrl) {
	using namespace nanogui;

	setLayout(new GroupLayout());
	setPosition(Vector2i(parent->width()/2.0f - 100.0f, parent->height()/2.0f - 100.0f));
	//setModal(true);

	auto configurables = ftl::config::list();
	const auto size = configurables.size();

	new Label(this, "Select Configurable","sans-bold");

	auto searchBox = new SearchBox(this, configurables);

	auto vscroll = new VScrollPanel(this);
	vscroll->setFixedHeight(300);
	auto buttons = new Widget(vscroll);
	buttons->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill));

	searchBox->setButtons(buttons);

	std::vector<std::string> configurable_titles(size);
	for (int i = 0; i < size; ++i) {
		ftl::URI uri(configurables[i]);
		std::string label = uri.getFragment();

		size_t pos = label.find_last_of("/");
		if (pos != std::string::npos) label = label.substr(pos+1);

		std::string parentName = configurables[i];
		size_t pos2 = parentName.find_last_of("/");
		if (pos2 != std::string::npos) parentName = parentName.substr(0,pos2);

		// FIXME: Does not indicated parent indentation ... needs sorting?

		if (i > 0 && parentName == configurables[i-1]) {
			ftl::URI uri(configurables[i-1]);
			configurable_titles[i-1] = std::string("[") + titleForURI(uri) + std::string("] ") + uri.getFragment();

			auto *prev = dynamic_cast<Button*>(buttons->childAt(buttons->childCount()-1));
			prev->setCaption(configurable_titles[i-1]);
			prev->setBackgroundColor(nanogui::Color(0.3f,0.3f,0.3f,1.0f));
			prev->setTextColor(nanogui::Color(1.0f,1.0f,1.0f,1.0f));
			prev->setIconPosition(Button::IconPosition::Left);
			prev->setIcon(ENTYPO_ICON_FOLDER);
		}

		configurable_titles[i] = label;

		auto itembutton = new nanogui::Button(buttons, configurable_titles[i]);
		std::string c = configurables[i];
		itembutton->setTooltip(c);
		itembutton->setBackgroundColor(nanogui::Color(0.9f,0.9f,0.9f,0.9f));
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
		configurable->refresh();
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

