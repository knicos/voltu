/**
 * @file soundctrl.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 * @author Nicolas Pope
 */

#pragma once

#include <nanogui/entypo.h>
#include <ftl/audio/mixer.hpp>

#include "popupbutton.hpp"

namespace ftl {
namespace gui2 {

class VolumeButton : public ftl::gui2::PopupButton {
public:
	VolumeButton(nanogui::Widget *parent, ftl::audio::StereoMixerF<100> *mixer);
	virtual ~VolumeButton();

	// callback, new value passed in argument
	void setCallback(std::function<void(float)> cb);

	// set value (updates slider value and highlight and changes icon)
	void setValue(float v);
	float value();

	// get/set mute status (changes volume highlight color and icon)
	void setMuted(bool v);
	bool muted();

	virtual bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) override;
	virtual bool scrollEvent(const nanogui::Vector2i &p, const nanogui::Vector2f &rel) override;

	// icons: 3 levels and muted
	int ICON_VOLUME_3 = ENTYPO_ICON_SOUND; // [67, 100]
	int ICON_VOLUME_2 = ENTYPO_ICON_SOUND; // [33,67)
	int ICON_VOLUME_1 = ENTYPO_ICON_SOUND; // [0,33)
	int ICON_MUTED = ENTYPO_ICON_SOUND_MUTE;

private:
	void update();

	nanogui::Slider* slider_;
	std::function<void(float)> cb_;

	ftl::audio::StereoMixerF<100> *mixer_;

	float scroll_step_ = 0.02f;
	float value_;
	bool muted_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
