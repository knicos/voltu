#ifndef _FTL_GUI_CFGWINDOW_HPP_
#define _FTL_GUI_CFGWINDOW_HPP_

#include <nanogui/window.h>
#include <nanogui/formhelper.h>

#include <ftl/master.hpp>
#include <ftl/uuid.hpp>
#include <ftl/net_configurable.hpp>

namespace ftl {
namespace gui {

/**
 * Allow configurable editing.
 */
class ConfigWindow : public nanogui::Window {
	public:
	ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl);
	~ConfigWindow();

	private:
	ftl::ctrl::Master *ctrl_;
	
	void _buildForm(const std::string &uri);
	void _addElements(nanogui::FormHelper *form, const std::string &suri);
	bool exists(const std::string &uri);
};

}
}

#endif  // _FTL_GUI_CFGWINDOW_HPP_
