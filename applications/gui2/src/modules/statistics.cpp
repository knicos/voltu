#include "statistics.hpp"

#include "../screen.hpp"
#include "../views/statistics.hpp"

#include <ftl/streams/builder.hpp>
#include <ftl/streams/netstream.hpp>

#include <nanogui/entypo.h>

#include <loguru.hpp>

using ftl::gui2::Statistics;
using ftl::gui2::StatisticsPanel;

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}

void Statistics::update(double delta) {
	time_count_ += delta;
	if (time_count_ > 1.0) {
		float bitrate = ftl::stream::Net::getRequiredBitrate();
		if (bitrate > 0.0f) {
			getJSON(StatisticsPanel::PERFORMANCE_INFO)["Bitrate"] = to_string_with_precision(bitrate, 1) + std::string("Mbit/s");
		}
		time_count_ = 0.0;
	}
}

void Statistics::init() {
	/**
	 * TODO: store all values in hash table and allow other modules to
	 * add/remove items/groups.
	 */

	widget = new ftl::gui2::StatisticsWidget(screen, this);
	widget->setVisible(value("visible", false));
	auto button = screen->addButton(ENTYPO_ICON_INFO);
	button->setTooltip("Show Information");
	button->setCallback([this, button](){
		button->setPushed(false);
		widget->setVisible(!widget->visible());
	});

	button->setVisible(true);
}

void Statistics::setCursor(nanogui::Cursor c) {
	widget->setCursor(c);
}

/*void Statistics::set(const std::string &key, const std::string& value) {
	text_[key] = value;
}

void Statistics::set(const std::string &key, float value, const std::string& unit) {
	text_[key] = to_string_with_precision(value, 3) + unit;
}

std::vector<std::string> Statistics::get() {
	std::vector<std::string> res;
	res.reserve(text_.size());
	for (auto& [k, v] : text_) {
		res.push_back(k + ": " +v );
	}
	return res;
}*/

nlohmann::json &Statistics::getJSON(StatisticsPanel p) {
	return groups_[p].json;
}

void Statistics::show(StatisticsPanel p, bool visible) {
	groups_[p].visible = visible;
}

void Statistics::hide(StatisticsPanel p) {
	groups_[p].visible = false;
}

bool Statistics::isVisible(StatisticsPanel p) {
	return groups_[p].visible;
}

std::vector<std::pair<StatisticsPanel, const nlohmann::json &>> Statistics::get() const {
	std::vector<std::pair<StatisticsPanel, const nlohmann::json &>> results;

	for (const auto &i : groups_) {
		results.emplace_back(i.first, i.second.json);
	}

	return results;
}
