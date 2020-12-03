/**
 * @file popupbutton.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#pragma once
#include <nanogui/button.h>
#include <nanogui/popup.h>

namespace ftl {
namespace gui2 {

/**
 * Patched version of nanogui::PopopButton with destructor which also removes
 * popup window on destruction.
 *
 * \class PopupButton popupbutton.h nanogui/popupbutton.h
 *
 * \brief Button which launches a popup widget.
 *
 * \remark
 *     This class overrides \ref nanogui::Widget::mIconExtraScale to be ``0.8f``,
 *     which affects all subclasses of this Widget.  Subclasses must explicitly
 *     set a different value if needed (e.g., in their constructor).
 */
class PopupButton : public nanogui::Button {
public:
	PopupButton(nanogui::Widget *parent, const std::string &caption = "",
				int buttonIcon = 0);
	virtual ~PopupButton();

	void setChevronIcon(int icon) { mChevronIcon = icon; }
	int chevronIcon() const { return mChevronIcon; }

	void setSide(nanogui::Popup::Side popupSide);
	nanogui::Popup::Side side() const { return mPopup->side(); }

	nanogui::Popup *popup() { return mPopup; }
	const nanogui::Popup *popup() const { return mPopup; }

	virtual void draw(NVGcontext* ctx) override;
	virtual nanogui::Vector2i preferredSize(NVGcontext *ctx) const override;
	virtual void performLayout(NVGcontext *ctx) override;

	virtual void save(nanogui::Serializer &s) const override;
	virtual bool load(nanogui::Serializer &s) override;

protected:
	nanogui::Popup *mPopup;
	int mChevronIcon;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
