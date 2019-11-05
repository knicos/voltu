#ifndef _FTL_GUI_CFGWINDOW_HPP_
#define _FTL_GUI_CFGWINDOW_HPP_

#include <nanogui/window.h>
#include <ftl/master.hpp>
#include <ftl/uuid.hpp>

#include <nanogui/formhelper.h>
#include <ftl/net_configurable.hpp>

namespace ftl {
namespace gui {

/**
 * Allow configurable editing.
 */
class ConfigWindow : public nanogui::Window {
	public:
	ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl, const ftl::UUID &peer);
	ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl, const std::optional<ftl::UUID> &peer = std::nullopt);
	~ConfigWindow();

	private:
	/*
	References holds the pointers to a NetConfigurable and all its members so that
	they can all be returned from _addElements() and then simultaneously deleted
	as the form is closed.
	*/
	class References;
	ftl::ctrl::Master *ctrl_;
	std::optional<ftl::UUID> peer_;
	std::vector<std::string> configurables_;
	
	void _buildForm(const std::string &uri);
	std::vector<References *> _addElements(nanogui::FormHelper *form, ftl::Configurable &nc, const std::string &suri, std::function<ftl::Configurable*(const std::string*, std::vector<References *>&)> construct);
	bool exists(const std::string &uri);
};

}
}

#endif  // _FTL_GUI_CFGWINDOW_HPP_
