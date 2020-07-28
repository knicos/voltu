#pragma once

#include "../module.hpp"
#include "../screen.hpp"

#include "../views/addsource.hpp"

namespace ftl {
namespace gui2 {

/**
 * Controller for adding sources etc.
 */
class AddCtrl : public Module {
public:
	using Module::Module;
	virtual ~AddCtrl();

	virtual void init() override;
	virtual void show();
	void disposeWindow();

	ftl::Configurable *add(const std::string &uri);

	std::vector<std::string> getHosts();
	std::set<ftl::stream::SourceInfo> getRecent();
	std::vector<std::string> getNetSources();
	std::vector<std::string> getFileSources();
	std::vector<std::string> getDeviceSources();
	std::vector<std::string> getGroups();
	std::string getSourceName(const std::string &uri);
	bool isSourceActive(const std::string &uri);

	inline ftl::stream::Feed *feed() { return io->feed(); }


private:
	nanogui::ToolButton *button;
	ftl::gui2::AddSourceWindow *window = nullptr;
};

}
}
