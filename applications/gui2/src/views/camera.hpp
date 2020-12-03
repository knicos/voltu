/**
 * @file camera.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 * @author Nicolas Pope
 */

#pragma once

#include "../view.hpp"

#include <ftl/utility/gltexture.hpp>

#include "../widgets/window.hpp"
#include "../widgets/soundctrl.hpp"
#include "../widgets/imageview.hpp"
#include "../widgets/popupbutton.hpp"
#include "../modules/camera_tools.hpp"

namespace ftl {
namespace gui2 {

class Camera;
class MediaPanel;
class CameraView;


struct ToolGroupData {
	nanogui::Button::Flags type;
	std::unordered_set<ftl::gui2::Tools> active;
	std::unordered_set<ftl::gui2::Tools> tools;
};

class ToolPanel : public FixedWindow {
public:
	ToolPanel(nanogui::Widget *parent, Camera* ctrl, CameraView* view);
	virtual ~ToolPanel();

	void setAvailable(const std::unordered_set<ftl::gui2::Tools> &);
	void setEnabled(const std::unordered_set<ftl::gui2::Tools> &);
	void enable(const std::unordered_set<ftl::gui2::Tools> &);
	void disable(const std::unordered_set<ftl::gui2::Tools> &);

	void draw(NVGcontext *ctx) override;

	//inline ftl::gui2::Tools activeTool() const { return active_; }
	bool isActive(ftl::gui2::Tools);
	void setTool(ftl::gui2::Tools tool);

	inline void addCallback(const std::function<bool(ftl::gui2::Tools)> &cb) { callbacks_.push_back(cb); }

private:
	Camera* ctrl_;
	CameraView* view_;
	nanogui::Widget *container_;
	std::unordered_map<ftl::gui2::Tools, nanogui::Button*> buttons_;
	std::unordered_map<ftl::gui2::ToolGroup, ftl::gui2::ToolGroupData> group_data_;
	std::unordered_map<ftl::gui2::Tools, ftl::gui2::ToolGroup> group_map_;

	std::list<std::function<bool(ftl::gui2::Tools)>> callbacks_;

	nanogui::Widget *_addGroup(ftl::gui2::ToolGroup group, nanogui::Button::Flags type, const std::unordered_set<ftl::gui2::Tools> &tools);
	void _addButton(nanogui::Widget *, ftl::gui2::Tools, int icon, const std::string &tooltip);
	void _addButton(ftl::gui2::PopupButton *parent, ftl::gui2::Tools, const std::string &label);
	ftl::gui2::PopupButton *_addButton(nanogui::Widget *, std::unordered_set<ftl::gui2::Tools> tools, int icon, const std::string &tooltip);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CameraView : public View {
public:
	CameraView(Screen* parent, Camera* ctrl);
	virtual ~CameraView();

	virtual void draw(NVGcontext* ctx) override;
	virtual void performLayout(NVGcontext* ctx) override;
	virtual bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) override;
	virtual bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) override;

	void refresh();
	void setZoom(bool enable);
	void setPan(bool enable);

	void setStereo(bool v);

protected:
	bool enable_zoom_;
	bool enable_pan_;
	Camera* ctrl_;
	MediaPanel* panel_;
	ToolPanel* tools_;
	FTLImageView* imview_;
	nanogui::Window *context_menu_;

private:
	StereoImageView* stereoim_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
