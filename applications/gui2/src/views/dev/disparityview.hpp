#pragma once

#include "../../view.hpp"

#include <ftl/utility/gltexture.hpp>

#include "../../widgets/window.hpp"
#include "../../widgets/imageview.hpp"
#include "../../widgets/popupbutton.hpp"
#include "../../modules/camera_tools.hpp"

#include "../camera.hpp"

namespace ftl {
namespace gui2 {

class DisparityDev;

class DisparityView : public View {
public:
	DisparityView(Screen* parent, DisparityDev* ctrl);
	virtual ~DisparityView();

	virtual void draw(NVGcontext* ctx) override;
	virtual void performLayout(NVGcontext* ctx) override;
	virtual bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) override;
	virtual bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) override;

	void refresh();

protected:
	DisparityDev* ctrl_;
	//MediaPanel* panel_;
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
