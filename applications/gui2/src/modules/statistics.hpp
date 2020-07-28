#pragma once

#include "../module.hpp"
#include <nlohmann/json.hpp>

namespace ftl
{
namespace gui2
{

enum class StatisticsPanel {
	MEDIA_STATUS=0,			// Live or not?
	PERFORMANCE_INFO,		// Bitrate, fps etc
	STREAM_DATA,			// Channel info
	MEDIA_META,				// Name, device, capabilities
	CAMERA_DETAILS,			// Calibration info
	LOGGING					// Stream error and log messages
	// Chat, media name, ...
};

class Statistics : public Module {
public:
	using Module::Module;
	virtual void init() override;
	virtual void update(double delta) override;

	// not thread safe! (use only from gui thread or add lock)
	/*void set(const std::string &key, const std::string& value);
	void set(const std::string &key, int value);
	void set(const std::string &key, float value, const std::string& unit = "");*/

	nlohmann::json &getJSON(StatisticsPanel);

	void show(StatisticsPanel, bool visible=true);
	void hide(StatisticsPanel);
	bool isVisible(StatisticsPanel);

	void setCursor(nanogui::Cursor);

	//void remove(const std::string &key) { text_.erase(key); }

	std::vector<std::pair<StatisticsPanel, const nlohmann::json &>> get() const;

private:
	struct StatsGroup {
		// TODO: Other properties...
		nlohmann::json json; // = nlohmann::json::object_t();
		bool visible=true;
	};

	nanogui::Widget* widget;
	std::map<StatisticsPanel, StatsGroup> groups_;
	double time_count_=0.0;
};
}
}
