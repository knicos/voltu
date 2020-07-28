
#include <loguru.hpp>

#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/entypo.h>
#include <nanogui/formhelper.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/opengl.h>

#include <nlohmann/json.hpp>

#include <vector>
#include <string>

#include "config.hpp"
#include "../screen.hpp"
#include "../widgets/leftbutton.hpp"

using ftl::gui2::ConfigWindow;
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
			screen()->performLayout();
		}
	}

public:
	SearchBox(Widget *parent, std::vector<std::string> &configurables) : nanogui::TextBox(parent, ""), configurables_(configurables) {
		setAlignment(TextBox::Alignment::Left);
		setEditable(true);
		setPlaceholder("Search");
	}

	virtual ~SearchBox() {
	}

	bool keyboardEvent(int key, int scancode, int action, int modifier) {
		TextBox::keyboardEvent(key, scancode, action, modifier);
		_setVisible(mValueTemp);
		return true;
	}

	void setButtons(Widget *buttons) {
		buttons_ = buttons;
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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

ConfigWindow::ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl)
		: nanogui::Window(parent, "Settings"), ctrl_(ctrl) {

	LOG(INFO) << __func__ << " (" << this << ")";

	using namespace nanogui;

	setTheme(dynamic_cast<ftl::gui2::Screen*>(screen())->getTheme("window_dark"));

	auto close = new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS);
	close->setTheme(dynamic_cast<ftl::gui2::Screen*>(screen())->getTheme("window_dark"));
	close->setBackgroundColor(theme()->mWindowHeaderGradientBot);
	close->setCallback([this](){ dispose();});

	setLayout(new GroupLayout(15, 6, 14, 10));
	setFixedWidth(400);
	setPosition(Vector2i(parent->width()/2.0f - 100.0f, parent->height()/2.0f - 100.0f));

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

	std::set<std::string> sorted_cfgs;
	sorted_cfgs.insert(configurables.begin(), configurables.end());

	for (auto &c : sorted_cfgs) {
		ftl::URI uri(c);

		std::string spacing = "";
		for (size_t i=0; i<uri.getPathLength(); ++i) {
			spacing += "    ";
		}

		//if (uri.getFragment().size() == 0) {
			std::string title = spacing + titleForURI(uri);
			//configurable_titles[i] = title;
			auto itembutton = new ftl::gui2::LeftButton(buttons, title);

			/*if (_isEmpty(c)) {
				itembutton->setEnabled(false);
			}*/
			itembutton->setTooltip(c);
			//itembutton->setBackgroundColor(nanogui::Color(0.9f,0.9f,0.9f,0.9f));
			itembutton->setCallback([this,c]() {
				_buildForm(c);
				setVisible(false);
				dispose();
			});
		//}
	}

	/*for (size_t i = 0; i < size; ++i) {
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
		if (_isEmpty(c)) {
			itembutton->setEnabled(false);
		}
		itembutton->setTooltip(c);
		itembutton->setBackgroundColor(nanogui::Color(0.9f,0.9f,0.9f,0.9f));
		itembutton->setCallback([this,c]() {
			_buildForm(c);
			setVisible(false);
			dispose();
		});
	}*/
}

ConfigWindow::~ConfigWindow() {
	LOG(INFO) << __func__ << " (" << this << ")";
}

bool ConfigWindow::_isEmpty(const std::string &uri) {
	// $id, $ref and tags always present
	return ftl::config::find(uri)->getConfig().size() <= 3;
}

void ConfigWindow::__addElements(nanogui::FormHelper *form, const std::string &suri) {
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
			const std::string suri = std::string(i.value().get<string>());
			__addElements(form, suri);
			continue;
		}

		if (i.value().is_boolean()) {
			string key = i.key();
			form->addVariable<bool>(i.key(), [data,key,suri](const bool &b){
				ftl::config::update(suri+"/"+key, b);
			}, [data,key]() -> bool {
				return data[key].get<bool>();
			});
		} else if (i.value().is_number_integer()) {
			string key = i.key();
			form->addVariable<int>(i.key(), [data,key,suri](const int &f){
				ftl::config::update(suri+"/"+key, f);
			}, [data,key]() -> int {
				return data[key].get<int>();
			});
		} else if (i.value().is_number_float()) {
			string key = i.key();
			form->addVariable<float>(i.key(), [data,key,suri](const float &f){
				ftl::config::update(suri+"/"+key, f);
			}, [data,key]() -> float {
				return data[key].get<float>();
			});
		} else if (i.value().is_string()) {
			string key = i.key();
			form->addVariable<string>(i.key(), [data,key,suri](const string &f){
				ftl::config::update(suri+"/"+key, f);
			}, [data,key]() -> string {
				return data[key].get<string>();
			});
		} else if (i.value().is_object()) {
			string key = i.key();
			string nuri;

			// Checking the URI with exists() prevents unloaded local configurations from being shown.
			//if (suri.find('#') != string::npos && exists(suri+string("/")+key)) {
			//	nuri = suri+string("/")+key;
			//} else
			if (exists(suri+string("/")+key)) {
				nuri = suri+string("/")+key;
			}

			if (!nuri.empty()) {
				nanogui::Window *window = form->window();
				auto button = form->addButton(key, [window, nuri]() {
					buildForm(window->screen(), nuri);
				});

				button->setIcon(ENTYPO_ICON_FOLDER);
				button->setIconPosition(nanogui::Button::IconPosition::Left);
				if (_isEmpty(nuri)) {
					button->setEnabled(false);
				}
			}
		}
	}
}

void ConfigWindow::_buildForm(const std::string &suri) {
	using namespace nanogui;

	/*ftl::URI uri(suri);

	FormHelper *form = new FormHelper(this->screen());
	form->addWindow(Vector2i(100,50), uri.getFragment());
	form->window()->setTheme(theme());

	__addElements(form, suri);

	// prevent parent window from being destroyed too early
	incRef();  // TODO: Is this needed? It isn't a parent window?

	auto close = new nanogui::Button(form->window()->buttonPanel(),	"",	ENTYPO_ICON_CROSS);
	close->setTheme(dynamic_cast<ftl::gui2::Screen*>(screen())->getTheme("window_dark"));
	close->setBackgroundColor(theme()->mWindowHeaderGradientBot);

	auto *window = form->window();

	close->setCallback([this, window](){
		window->dispose();
		decRef();
	});
	close->setBackgroundColor({80, 255});
	form->window()->screen()->performLayout();
	delete form;*/

	buildForm(screen(), suri);
}

static MUTEX config_mtx;
static std::unordered_map<std::string, nanogui::Window*> existing_configs;

// Static version
void ConfigWindow::buildForm(nanogui::Screen *screen, const std::string &suri) {
	using namespace nanogui;

	{
		UNIQUE_LOCK(config_mtx, lk);
		auto i = existing_configs.find(suri);
		if (i != existing_configs.end()) {
			screen->moveWindowToFront(i->second);
			return;
		}
	}

	ftl::URI uri(suri);

	FormHelper *form = new FormHelper(screen);
	form->addWindow(Vector2i(100,50), titleForURI(uri));
	//form->window()->setTheme(theme());

	{
		UNIQUE_LOCK(config_mtx, lk);
		existing_configs[suri] = form->window();
	}

	auto *window = form->window();
	window->setTheme(dynamic_cast<ftl::gui2::Screen*>(window->screen())->getTheme("window_dark"));
	window->setWidth(200);

	__addElements(form, suri);

	// prevent parent window from being destroyed too early
	//incRef();  // TODO: Is this needed? It isn't a parent window?

	auto close = new nanogui::Button(form->window()->buttonPanel(),	"",	ENTYPO_ICON_CROSS);
	close->setTheme(dynamic_cast<ftl::gui2::Screen*>(screen)->getTheme("window_dark"));
	//close->setBackgroundColor(theme()->mWindowHeaderGradientBot);

	close->setCallback([window, suri](){
		window->dispose();
		//decRef();
		UNIQUE_LOCK(config_mtx, lk);
		existing_configs.erase(suri);
	});
	close->setBackgroundColor({80, 255});
	form->window()->screen()->performLayout();
	delete form;
}

bool ConfigWindow::exists(const std::string &uri) {
	return ftl::config::find(uri) != nullptr;
}
