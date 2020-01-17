#include "record_window.hpp"

#include "screen.hpp"

#include <ftl/codecs/channels.hpp>

#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/combobox.h>
#include <nanogui/label.h>
#include <nanogui/textbox.h>
#include <nanogui/tabwidget.h>

using ftl::gui::RecordWindow;

RecordWindow::RecordWindow(nanogui::Widget *parent, ftl::gui::Screen *screen, const std::vector<ftl::gui::Camera *> &streams, ftl::gui::MediaPanel *media_panel)
        : nanogui::Window(parent, "Recording options") {
    using namespace nanogui;

    setLayout(new GroupLayout());

    new Label(this, "File name", "sans-bold");
    char timestamp[18];
	std::time_t t = std::time(NULL);
	std::strftime(timestamp, sizeof(timestamp), "%F-%H%M%S", std::localtime(&t));
    Widget *fileNameBox = new Widget(this);
    fileNameBox->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 6));
    auto fileName = new TextBox(fileNameBox, std::string(timestamp));
    fileName->setFixedWidth(350);
    fileName->setEditable(true);
    auto extension = new Label(fileNameBox, ".png", "sans-bold");
    new Label(this, "Select stream", "sans-bold");
    auto streamNames = std::vector<std::string>();
    streamNames.reserve(streams.size());
    std::optional<int> ix;
	int i=1;
    for (const auto s : streams) {
        if (s == screen->activeCamera()) {
            ix = std::optional<int>(streamNames.size());
        }
        // FIXME: Find alternative to source URI
        //streamNames.push_back(s->source()->getURI());
		streamNames.push_back(std::string("Stream")+std::to_string(i++));
    }
    auto streamSelect = new ComboBox(this, streamNames);
    // TODO: The function availableChannels() only finds those channels that
    // have been set in camera.cpp. The only channels that are set in
    // camera.cpp currently are Colour and Depth. This means that currently,
    // the list of channels returned by availableChannels() is not accurate
    // and should be fixed.
    TabWidget *tabWidget = add<TabWidget>();
    tabWidget->setFixedWidth(400);
    auto snapshot2D = tabWidget->createTab("2D snapshot");
    auto recording2D = tabWidget->createTab("2D recording");
    auto snapshot3D = tabWidget->createTab("3D snapshot");
    auto recording3D = tabWidget->createTab("3D recording");

    snapshot2D->setLayout(new GroupLayout());
    recording2D->setLayout(new GroupLayout());
    snapshot3D->setLayout(new GroupLayout());
    recording3D->setLayout(new GroupLayout());

    // Set the file name extension based on the type of recording chosen.
    tabWidget->setCallback([tabWidget,snapshot2D,extension](int ix) {
        if (tabWidget->tab(ix) == snapshot2D) {
            extension->setCaption(".png");
        } else {
            extension->setCaption(".ftl");
        }
    });

    tabWidget->setActiveTab(0);

    new Label(recording2D, "Select channel (in addition to Left)", "sans-bold");
    auto recordingChannel = recording2D->add<ComboBox>();
    auto streamCallback = [this,streams,recordingChannel](int ix) {
        channels_ = std::vector<ftl::codecs::Channel>();
        channel_names_ = std::vector<std::string>();
        ftl::codecs::Channels availableChannels = streams[ix]->availableChannels();
        for (auto c : availableChannels) {
            channels_.push_back(c);
            channel_names_.push_back(ftl::codecs::name(c));
        }
        recordingChannel->setItems(channel_names_);
    };
    streamSelect->setCallback(streamCallback);

    // Set the selection to the active stream and set the channel list
    // to be the channels available in that stream. The callback must
    // be called explicitly, since setSelectedIndex() does not trigger it.
    if (ix) {
        streamSelect->setSelectedIndex(ix.value());
        streamCallback(ix.value());
    }

    Widget *actionButtons = new Widget(this);
    actionButtons->setLayout(new BoxLayout(Orientation::Horizontal));
    auto button = new Button(actionButtons, "Start");
    button->setCallback([this,streams,streamSelect,screen,media_panel,fileName,extension,tabWidget,snapshot2D,recording2D,snapshot3D,recording3D,recordingChannel]() {
        // Check the chosen stream type and channels, then record them.
        std::string name = fileName->value() + extension->caption();
        auto stream = streams[streamSelect->selectedIndex()];
        auto tab = tabWidget->tab(tabWidget->activeTab());
        if (tab == snapshot2D) {
            stream->snapshot(name);
        } else if (tab == recording2D) {
            stream->setChannel(channels_[recordingChannel->selectedIndex()]);
            screen->setActiveCamera(stream);
            //media_panel->startRecording2D(stream, name);
        } else if (tab == snapshot3D) {
            //media_panel->snapshot3D(stream, name);
        } else if (tab == recording3D) {
            //media_panel->startRecording3D(stream, name);
        }
        dispose();
        media_panel->recordWindowClosed();
    });
    button = new Button(actionButtons, "Cancel");
    button->setCallback([this,media_panel]() {
        dispose();
        media_panel->recordWindowClosed();
    });
}

RecordWindow::~RecordWindow() {
    
}
