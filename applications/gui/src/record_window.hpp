#include <nanogui/window.h>

#include "camera.hpp"
#include "media_panel.hpp"

namespace ftl {
namespace gui {

class RecordWindow : public nanogui::Window {
    public:
    explicit RecordWindow(nanogui::Widget *parent, ftl::gui::Screen *screen, const std::vector<ftl::gui::Camera *> &streams, ftl::gui::MediaPanel *media_panel);
    ~RecordWindow();

    private:
    std::vector<ftl::codecs::Channel> channels_;
    std::vector<std::string> channel_names_;
};

}
}