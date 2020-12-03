/**
 * @file statistics.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once

#include <nanogui/widget.h>
#include <nanogui/window.h>

namespace ftl
{
namespace gui2
{

class Statistics;

class StatisticsWidget : public nanogui::Window {
public:
	StatisticsWidget(nanogui::Widget *parent, Statistics* ctrl);
	virtual void draw(NVGcontext *ctx);

	bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) override { return false; }

private:
	Statistics* ctrl_;
	int last_stats_count_;
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
