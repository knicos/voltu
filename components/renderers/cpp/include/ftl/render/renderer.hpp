#ifndef _FTL_RENDER_RENDERER_HPP_
#define _FTL_RENDER_RENDERER_HPP_

#include <ftl/configurable.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace render {

/**
 * Abstract class for all renderers. A renderer takes some 3D scene and
 * generates a virtual camera perspective of that scene. The scene might be
 * based upon a point cloud, or an entirely virtual mesh or procedural scene.
 * It is intended that multiple scenes can be rendered into a single virtual
 * view using a compositing renderer, such a renderer accepting any kind of
 * renderer for compositing and hence relying on this base class.
 */
class Renderer : public ftl::Configurable {
    public:
    explicit Renderer(nlohmann::json &config) : Configurable(config) {};
    virtual ~Renderer() {};

    /**
     * Generate a single virtual frame. The frame takes its pose and calibration
	 * from the output frame pose and calibration channels.
     */
    virtual bool render(ftl::rgbd::FrameSet &, ftl::rgbd::Frame &, ftl::codecs::Channel, const Eigen::Matrix4d &)=0;
};

}
}

#endif  // _FTL_RENDER_RENDERER_HPP_
