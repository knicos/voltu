#pragma once

#include "../view.hpp"

#include <ftl/utility/gltexture.hpp>

#include "../widgets/window.hpp"
#include "../widgets/soundctrl.hpp"
#include "../widgets/imageview.hpp"

namespace ftl {
namespace gui2 {

class Camera;
class MediaPanel;

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
	FTLImageView* imview_;
	nanogui::Window *context_menu_;

private:
	StereoImageView* stereoim_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
