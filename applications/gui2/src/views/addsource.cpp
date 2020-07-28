#include "addsource.hpp"
#include "../modules/addsource.hpp"

#include "../widgets/combobox.hpp"

#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/tabwidget.h>
#include <nanogui/formhelper.h>

#include <loguru.hpp>


using ftl::gui2::AddSourceWindow;

AddSourceWindow::AddSourceWindow(nanogui::Widget* parent, AddCtrl *ctrl) :
		nanogui::Window(parent, ""), ctrl_(ctrl) {

	using namespace nanogui;

	auto t = dynamic_cast<ftl::gui2::Screen*>(screen())->getTheme("window_dark");
	setTheme(t);

	//setFixedWidth(500);
	setFixedSize(Vector2i(500,300));
	setLayout(new nanogui::BoxLayout
				(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 20, 10));

	setPosition(Vector2i(parent->width()/2.0f - fixedWidth()/2.0f, parent->height()/2.0f - fixedHeight()/2.0f));

	auto close = new nanogui::Button(buttonPanel(), "", ENTYPO_ICON_CROSS);
	close->setTheme(dynamic_cast<ftl::gui2::Screen*>(screen())->getTheme("window_dark"));
	close->setBackgroundColor(theme()->mWindowHeaderGradientBot);
	close->setCallback([this](){ this->close();});

	auto *title = new Label(this, "Add Source", "sans-bold");
	title->setFontSize(28);

	tabs_ = new TabWidget(this);

	auto *recent_tab = tabs_->createTab("Recent");
	recent_tab->setLayout(new nanogui::BoxLayout
				(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0, 0));
	VScrollPanel *vscroll = new VScrollPanel(recent_tab);
	vscroll->setFixedHeight(200);
	Widget *recentscroll = new Widget(vscroll);
	recentscroll->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 4));

	auto *group_tab = tabs_->createTab("Groups");
	group_tab->setLayout(new nanogui::BoxLayout
				(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0, 0));
	vscroll = new VScrollPanel(group_tab);
	vscroll->setFixedHeight(200);
	Widget *groupscroll = new Widget(vscroll);
	groupscroll->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 4));

	auto *dev_tab = tabs_->createTab("Devices");
	dev_tab->setLayout(new nanogui::BoxLayout
				(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0, 0));
	vscroll = new VScrollPanel(dev_tab);
	vscroll->setFixedHeight(200);
	Widget *devscroll = new Widget(vscroll);
	devscroll->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 4));

	auto *host_tab = tabs_->createTab("Hosts");
	host_tab->setLayout(new nanogui::BoxLayout
				(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0, 0));
	vscroll = new VScrollPanel(host_tab);
	vscroll->setFixedHeight(200);
	Widget *hostscroll = new Widget(vscroll);
	hostscroll->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 4));

	auto *stream_tab = tabs_->createTab("Streams");
	stream_tab->setLayout(new nanogui::BoxLayout
				(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0, 0));
	vscroll = new VScrollPanel(stream_tab);
	vscroll->setFixedHeight(200);
	Widget *streamscroll = new Widget(vscroll);
	streamscroll->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 4));

	auto *file_tab = tabs_->createTab("Files");
	file_tab->setLayout(new nanogui::BoxLayout
				(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 0, 0));
	vscroll = new VScrollPanel(file_tab);
	vscroll->setFixedHeight(200);
	Widget *filescroll = new Widget(vscroll);
	filescroll->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 4));

	tab_items_.resize(6);
	tab_items_[0] = recentscroll;
	tab_items_[1] = groupscroll;
	tab_items_[2] = devscroll;
	tab_items_[3] = hostscroll;
	tab_items_[4] = streamscroll;
	tab_items_[5] = filescroll;

	uptodate_.test_and_set();
	rebuild();
	tabs_->setActiveTab(0);

	new_source_handle_ = ctrl_->feed()->onNewSources([this](const std::vector<std::string> &srcs) {
		UNIQUE_LOCK(mutex_, lk);
		uptodate_.clear();
		return true;
	});
}

AddSourceWindow::~AddSourceWindow() {

}

nanogui::Button *AddSourceWindow::_addButton(const std::string &s, nanogui::Widget *parent, bool hide) {
	using namespace nanogui;

	ftl::URI uri(s);
	int icon = 0;
	switch (uri.getScheme()) {
	case ftl::URI::SCHEME_DEVICE		: icon = ENTYPO_ICON_CAMERA; break;
	case ftl::URI::SCHEME_FILE			: icon = ENTYPO_ICON_FOLDER_VIDEO; break;
	case ftl::URI::SCHEME_FTL			: icon = ENTYPO_ICON_CLOUD; break;
	case ftl::URI::SCHEME_WS			:
	case ftl::URI::SCHEME_TCP			: icon = ENTYPO_ICON_CLASSIC_COMPUTER; break;
	case ftl::URI::SCHEME_GROUP			: icon = ENTYPO_ICON_MERGE; break;
	default: break;
	}

	auto *button = new Button(parent, ctrl_->getSourceName(s), icon);
	if (ctrl_->isSourceActive(s)) {
		button->setBackgroundColor(Color(0, 255, 0, 25));
	}

	button->setIconPosition(Button::IconPosition::Left);
	button->setIconExtraScale(1.2);
	button->setFontSize(18);
	button->setTooltip(s);

	button->setCallback([this, uri = s, hide]() {
		if (hide) close();
		ctrl_->add(uri);
	});

	return button;
}

void AddSourceWindow::rebuild() {
	using namespace nanogui;

	for (auto *w : tab_items_) {
		while (w->childCount() > 0) w->removeChild(w->childCount()-1);
	}
	
	Button *button;

	auto srcs = ctrl_->getRecent();
	for (auto &s : srcs) {
		_addButton(s.uri, tab_items_[0]);
	}

	auto groups = ctrl_->getGroups();
	for (auto &s : groups) {
		_addButton(s, tab_items_[1]);
	}

	auto devsrcs = ctrl_->getDeviceSources();
	for (auto &s : devsrcs) {
		_addButton(s, tab_items_[2]);
	}

	auto *host_menu = new Widget(tab_items_[3]);
	host_menu->setLayout(new BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Maximum, 5,4));

	button = new Button(host_menu, "Add", ENTYPO_ICON_PLUS);
	button->setFontSize(18);
	button->setTooltip("Connect to a new machine");
	button->setCallback([this]() {
		FormHelper *fh = new FormHelper(screen());
		auto *win = fh->addWindow(Vector2i(10,10), "Add Host");
		win->center();
		win->setTheme(dynamic_cast<ftl::gui2::Screen*>(win->screen())->getTheme("window_dark"));
		//win->setWidth(200);
		fh->addVariable<std::string>("URI", [this,win](const std::string &v) {
			try {
				ctrl_->add(v);
			} catch (const ftl::exception &e) {
				LOG(ERROR) << "Add failed: " << e.what();
			}
			win->dispose();
		}, [this]() {
			return "";
		})->setFixedWidth(150);
		win->screen()->performLayout();
		delete fh;
	});

	button = new Button(host_menu, "Clear", ENTYPO_ICON_CYCLE);
	button->setFontSize(18);
	button->setTooltip("Clear host history");
	button->setCallback([this]() {
		ctrl_->feed()->clearHostHistory();
		uptodate_.clear();
	});

	auto hostsrcs = ctrl_->getHosts();
	for (auto &s : hostsrcs) {
		_addButton(s, tab_items_[3], false);
	}

	auto streamsrcs = ctrl_->getNetSources();
	for (auto &s : streamsrcs) {
		_addButton(s, tab_items_[4]);
	}

	auto *file_menu = new Widget(tab_items_[5]);
	file_menu->setLayout(new BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Maximum, 5,4));

	button = new Button(file_menu, "Open", ENTYPO_ICON_PLUS);
	button->setFontSize(18);
	button->setTooltip("Open FTL File");
	button->setCallback([this]() {
		try {
			std::string filename = file_dialog({ {"ftl", "FTL Captures"} }, false);
			if (filename.size() > 0 && filename[0] == '/') {
				filename = std::string("file://") + filename;
			} else {
				filename = std::string("file:///") + filename;
			}
#ifdef WIN32
			auto p = filename.find_first_of('\\');
			while (p != std::string::npos) {
				filename[p] = '/';
				p = filename.find_first_of('\\');
			}
#endif
			ctrl_->add(filename);
		} catch (const std::exception &e) {
			LOG(ERROR) << "File load exception: " << e.what();
		}
		close();
	});

	button = new Button(file_menu, "Clear", ENTYPO_ICON_CYCLE);
	button->setFontSize(18);
	button->setTooltip("Clear file history");
	button->setCallback([this]() {
		ctrl_->feed()->clearFileHistory();
		uptodate_.clear();
	});

	auto filesrcs = ctrl_->getFileSources();
	for (auto &s : filesrcs) {
		_addButton(s, tab_items_[5]);
	}
}

void AddSourceWindow::close() {
	setVisible(false);
	//dispose();
	ctrl_->disposeWindow();
}

void AddSourceWindow::draw(NVGcontext *ctx) {
	{
		UNIQUE_LOCK(mutex_, lk);
		if (!uptodate_.test_and_set()) {
			tabs_->requestFocus();  // Must ensure focus item not deleted
			rebuild();
			screen()->performLayout();
		}
	}

	nanogui::Window::draw(ctx);
}
