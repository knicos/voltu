#ifndef _FTL_GUI_CTRLWINDOW_HPP_
#define _FTL_GUI_CTRLWINDOW_HPP_

#include <nanogui/window.h>
#include <ftl/master.hpp>
#include <ftl/uuid.hpp>

namespace ftl {
namespace gui {

/**
 * Manage connected nodes and add new connections.
 */
class ControlWindow : public nanogui::Window {
	public:
	ControlWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl);
	~ControlWindow();

	private:
	ftl::ctrl::Master *ctrl_;
	std::vector<ftl::config::json_t> node_details_;
	std::vector<std::string> node_titles_;
	int active_ix_;
	std::string add_node_uri_;

	void _updateDetails();
	void _changeActive(int);
	ftl::UUID _getActiveID();
	void _addNode();
};

}
}

#endif  // _FTL_GUI_CTRLWINDOW_HPP_
