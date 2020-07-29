#pragma once

#include <nanogui/window.h>
#include <ftl/handle.hpp>
#include <ftl/threads.hpp>


namespace ftl {
namespace gui2 {

class AddCtrl;

/**
 * Add source dialog
 */
class AddSourceWindow : public nanogui::Window {
	public:
	AddSourceWindow(nanogui::Widget *parent, AddCtrl *ctrl);
	virtual ~AddSourceWindow();

	virtual void draw(NVGcontext *ctx);

private:
	AddCtrl *ctrl_;
	void close();
	void rebuild();

	nanogui::Button *_addButton(const std::string &s, nanogui::Widget *parent, bool hide=true);

	ftl::Handle new_source_handle_;
	MUTEX mutex_;
	std::atomic_flag uptodate_;
	std::vector<nanogui::Widget*> tab_items_;
	nanogui::TabWidget *tabs_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
