#ifndef _FTL_GUI_MEDIAPANEL_HPP_
#define _FTL_GUI_MEDIAPANEL_HPP_

#include <nanogui/window.h>

namespace ftl {

namespace rgbd {
class SnapshotStreamWriter;
}

namespace gui {

class Screen;

class MediaPanel : public nanogui::Window {
    public:
    MediaPanel(ftl::gui::Screen *);
    ~MediaPanel();

    private:
    ftl::gui::Screen *screen_;
    bool paused_;
    ftl::rgbd::SnapshotStreamWriter *writer_;
};

}
}

#endif  // _FTL_GUI_MEDIAPANEL_HPP_
