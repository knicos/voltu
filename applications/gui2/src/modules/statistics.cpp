/**
 * @file statistics.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 * @author Sebastian Hahta
 */

#include "statistics.hpp"

#include "../screen.hpp"
#include "../views/statistics.hpp"

#include <ftl/streams/builder.hpp>
#include <ftl/streams/netstream.hpp>

#include <nanogui/entypo.h>

#include <loguru.hpp>

#include <nvml.h>

#ifdef WIN32
#pragma comment(lib, "nvml")
#endif

using ftl::gui2::Statistics;
using ftl::gui2::StatisticsPanel;

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}

Statistics::~Statistics() {
	nvmlShutdown();
}

void Statistics::update(double delta) {
	time_count_ += delta;
	if (time_count_ > 1.0) {
		float bitrate = ftl::stream::Net::getRequiredBitrate();
		if (bitrate > 0.0f) {
			getJSON(StatisticsPanel::PERFORMANCE_INFO)["Bitrate"] = to_string_with_precision(bitrate, 1) + std::string("Mbit/s");
		}
		time_count_ = 0.0;

		size_t gpu_free_mem;
		size_t gpu_total_mem;
		cudaSafeCall(cudaMemGetInfo(&gpu_free_mem, &gpu_total_mem));
		float gpu_mem = 1.0f - (float(gpu_free_mem) / float(gpu_total_mem));
		getJSON(StatisticsPanel::PERFORMANCE_INFO)["GPU Memory"] = to_string_with_precision(gpu_mem*100.0f, 1) + std::string("%");

		nvmlDevice_t device;
        auto result = nvmlDeviceGetHandleByIndex(0, &device);
		nvmlUtilization_st device_utilization;
        result = nvmlDeviceGetUtilizationRates(device, &device_utilization);
		getJSON(StatisticsPanel::PERFORMANCE_INFO)["GPU Usage"] = std::to_string(device_utilization.gpu) + std::string("%");

		unsigned int decode_util;
		unsigned int decode_period;
		result = nvmlDeviceGetDecoderUtilization(device, &decode_util, &decode_period);
		getJSON(StatisticsPanel::PERFORMANCE_INFO)["GPU Decoder"] = std::to_string(decode_util) + std::string("%");

		// Doesn't seem to work
		unsigned int encoder_sessions=0;
		unsigned int encoder_fps;
		unsigned int encoder_latency;
		result = nvmlDeviceGetEncoderStats(device, &encoder_sessions, &encoder_fps, &encoder_latency);

		unsigned int encoder_util;
		unsigned int encoder_period;
		result = nvmlDeviceGetEncoderUtilization(device, &encoder_util, &encoder_period);
		getJSON(StatisticsPanel::PERFORMANCE_INFO)["GPU Encoder"] = std::to_string(encoder_util) + std::string("% (") + std::to_string(encoder_sessions) + std::string(")");
	}
}

void Statistics::init() {
	auto result = nvmlInit();
    if (result != NVML_SUCCESS) throw FTL_Error("No NVML");

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
