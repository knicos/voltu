#ifndef _FTL_GUI_CFGWINDOW_HPP_
#define _FTL_GUI_CFGWINDOW_HPP_

#include <nanogui/window.h>
#include <ftl/master.hpp>
#include <ftl/uuid.hpp>

#include <nanogui/formhelper.h>

namespace ftl {
namespace gui {

/**
 * Allow configurable editing.
 */
class ConfigWindow : public nanogui::Window {
	public:
	ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl, const ftl::UUID &peer);
	~ConfigWindow();

	private:
	ftl::ctrl::Master *ctrl_;
	ftl::UUID peer_;
	std::vector<std::string> configurables_;
	
	void _buildForm(const std::string &uri, ftl::config::json_t data);
	void _addElements(nanogui::FormHelper *form, const std::string &suri, const ftl::config::json_t &data);
};

}
}

#endif  // _FTL_GUI_CFGWINDOW_HPP_
