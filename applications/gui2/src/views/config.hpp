#pragma once

#include <nanogui/window.h>
#include <nanogui/formhelper.h>

#include <ftl/master.hpp>
#include <ftl/uuid.hpp>
#include <ftl/net_configurable.hpp>

namespace ftl {
namespace gui2 {

/**
 * Allow configurable editing.
 */
class ConfigWindow : public nanogui::Window {
	public:
	ConfigWindow(nanogui::Widget *parent, ftl::ctrl::Master *ctrl);
	virtual ~ConfigWindow();

	static void buildForm(nanogui::Screen *screen, const std::string &uri);

private:
	ftl::ctrl::Master *ctrl_;

	static bool _isEmpty(const std::string &uri);
	void _buildForm(const std::string &uri);
	static void __addElements(nanogui::FormHelper *form, const std::string &suri);
	static bool exists(const std::string &uri);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
