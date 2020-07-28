/*
	Modification: Inherits from ftl::gui2::PopupButton

	NanoGUI was developed by Wenzel Jakob <wenzel.jakob@epfl.ch>.
	The nanogui::Widget drawing code is based on the NanoVG demo application
	by Mikko Mononen.

	All rights reserved. Use of this source code is governed by a
	BSD-style license that can be found in the LICENSE.txt file.
*/
/**
 * \file nanogui/combobox.h
 *
 * \brief Simple combo box nanogui::Widget based on a popup button.
 */

#pragma once

#include "popupbutton.hpp"

namespace ftl {
namespace gui2 {

/**
 * \class ComboBox combobox.h nanogui/combobox.h
 *
 * \brief Simple combo box nanogui::Widget based on a popup button.
 */
class NANOGUI_EXPORT ComboBox : public PopupButton {
public:
	/// Create an empty combo box
	ComboBox(nanogui::Widget *parent);

	/// Create a new combo box with the given items
	ComboBox(nanogui::Widget *parent, const std::vector<std::string> &items);

	/**
	 * \brief Create a new combo box with the given items, providing both short and
	 * long descriptive labels for each item
	 */
	ComboBox(nanogui::Widget *parent, const std::vector<std::string> &items,
			 const std::vector<std::string> &itemsShort);

	/// The callback to execute for this ComboBox.
	std::function<void(int)> callback() const { return mCallback; }

	/// Sets the callback to execute for this ComboBox.
	void setCallback(const std::function<void(int)> &callback) { mCallback = callback; }

	/// The current index this ComboBox has selected.
	int selectedIndex() const { return mSelectedIndex; }

	/// Sets the current index this ComboBox has selected.
	void setSelectedIndex(int idx);

	/// Sets the items for this ComboBox, providing both short and long descriptive lables for each item.
	void setItems(const std::vector<std::string> &items, const std::vector<std::string> &itemsShort);

	/// Sets the items for this ComboBox.
	void setItems(const std::vector<std::string> &items) { setItems(items, items); }

	/// The items associated with this ComboBox.
	const std::vector<std::string> &items() const { return mItems; }

	/// The short descriptions associated with this ComboBox.
	const std::vector<std::string> &itemsShort() const { return mItemsShort; }

	/// Handles mouse scrolling events for this ComboBox.
	virtual bool scrollEvent(const nanogui::Vector2i &p, const nanogui::Vector2f &rel) override;

	/// Saves the state of this ComboBox to the specified nanogui::Serializer.
	virtual void save(nanogui::Serializer &s) const override;

	/// Sets the state of this ComboBox from the specified nanogui::Serializer.
	virtual bool load(nanogui::Serializer &s) override;

protected:
	/// The items associated with this ComboBox.
	std::vector<std::string> mItems;

	/// The short descriptions of items associated with this ComboBox.
	std::vector<std::string> mItemsShort;

	/// The callback for this ComboBox.
	std::function<void(int)> mCallback;

	/// The current index this ComboBox has selected.
	int mSelectedIndex;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
