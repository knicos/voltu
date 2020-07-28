#pragma once

#include "../module.hpp"
#include "../screen.hpp"

#include "../views/config.hpp"

namespace ftl {
namespace gui2 {

/**
 * Controller for thumbnail view.
 */
class ConfigCtrl : public Module {
public:
	using Module::Module;
	virtual ~ConfigCtrl();

	virtual void init() override;
	virtual void show();

	void show(const std::string &uri);

private:
	nanogui::ToolButton *button;
	ftl::gui2::ConfigWindow *window = nullptr;

	std::list<nanogui::FormHelper *> forms_;
};

}
}
