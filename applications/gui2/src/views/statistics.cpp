/**
 * @file statistics.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 * @author Sebastian Hahta
 */

#include "statistics.hpp"
#include "../modules/statistics.hpp"

#include <ftl/streams/builder.hpp>
#include <ftl/streams/netstream.hpp>
#include <ftl/render/colouriser.hpp>
#include <ftl/utility/string.hpp>

#include <nanogui/screen.h>
#include <nanogui/opengl.h>

#include <loguru.hpp>

using ftl::gui2::StatisticsWidget;
using std::string;

StatisticsWidget::StatisticsWidget(nanogui::Widget* parent, ftl::gui2::Statistics* ctrl) :
		nanogui::Window(parent,""), ctrl_(ctrl), last_stats_count_(0) {

	setWidth(parent->width()/2);
}

void StatisticsWidget::draw(NVGcontext *ctx) {
	int margin = 20;
	const auto &screenSize = screen()->size();
	float rowh = 10.0;
	int count = 0;

	setPosition({screenSize[0] - width() - margin, 0});
	setHeight(screenSize[1]);

	const auto pos = absolutePosition();
	auto panels = ctrl_->get();
	for (unsigned int i = 0; i < panels.size(); i++) {
		if (panels[i].second.is_structured()) {
			for (auto j : panels[i].second.items()) {
				std::string msg = j.key();

				auto colour = nanogui::Color(244, 244, 244, 255);
				int fsize = 15;
				int entypo = 0;
				
				if (j.value().is_object()) {
					const auto &val = j.value()["value"];

					if (j.value().contains("nokey")) {
						msg = "";
					}
					if (j.value().contains("colour")) {
						uchar4 cucol = ftl::render::parseCUDAColour(j.value()["colour"].get<std::string>());
						colour = nanogui::Color(cucol.x, cucol.y, cucol.z, 255);
					}
					if (j.value().contains("size")) {
						fsize = j.value()["size"].get<int>();
					}
					if (j.value().contains("icon")) {
						entypo = j.value()["icon"].get<int>();
					}

					if (val.is_string()) {
						if (msg.size() > 0) msg += std::string(": ");
						msg += val.get<std::string>();
					} else if (val.is_number()) {
						if (msg.size() > 0) msg += std::string(": ");
						msg += std::string(": ") + to_string_with_precision(val.get<float>(),2);
					}
				} else if (j.value().is_string()) {
					msg += std::string(": ") + j.value().get<std::string>();
				} else if (j.value().is_number()) {
					msg += std::string(": ") + to_string_with_precision(j.value().get<float>(),2);
				} else if (j.value().is_boolean()) {

				}

				rowh += float(fsize)+5.0f;

				nvgFontSize(ctx, fsize);
				nvgTextAlign(ctx, NVG_ALIGN_RIGHT);

				float tw = 0.0f;

				if (msg.size() > 0) {
					if (panels[i].first == ftl::gui2::StatisticsPanel::LOGGING) nvgFontFace(ctx, "sans");
					else nvgFontFace(ctx, "sans-bold");
					nvgFillColor(ctx, nanogui::Color(8, 8, 8, 255)); // shadow
					tw = nvgTextBounds(ctx, pos[0] + width(), rowh, msg.c_str(), nullptr, nullptr);
					nvgText(ctx, pos[0] + width(), rowh, msg.c_str(), nullptr);
					nvgFillColor(ctx, colour);
					nvgText(ctx, pos[0] + width() - 1, rowh - 1, msg.c_str(), nullptr);
					tw += 10;
				}
				if (entypo > 0) {
					auto icon = nanogui::utf8(entypo);
					nvgFontFace(ctx, "icons");
					nvgFontSize(ctx, float(fsize)*0.8f);
					nvgFillColor(ctx, nanogui::Color(8, 8, 8, 255)); // shadow
					nvgText(ctx, pos[0] + width() - tw, rowh, icon.data(), nullptr);
					nvgFillColor(ctx, colour);
					nvgText(ctx, pos[0] + width() - 1 - tw, rowh - 1, icon.data(), nullptr);
				}

				++count;
			}
		}
	}
}
