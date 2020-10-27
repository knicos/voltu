#ifndef _FTL_RENDER_RENDERER_HPP_
#define _FTL_RENDER_RENDERER_HPP_

#include <ftl/configurable.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/cuda_common.hpp>

namespace ftl {
namespace render {

enum class Stage {
	Finished,
	ReadySubmit,
	Blending
};

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
    explicit Renderer(nlohmann::json &config) : Configurable(config), stage_(Stage::Finished) {};
    virtual ~Renderer() {};

	/**
	 * Begin a new render. This clears memory, allocates buffers etc. The RGBD
	 * frame given as parameter is where the output channels are rendered to.
	 * The channel parameter is the render output channel which can either be
	 * Left (Colour) or Right (Colour 2). Using "Right" will also adjust the
	 * pose to the right eye position and use the right camera intrinsics. The
	 * pose and intrinsics also come from the given `Frame` object.
	 */
	virtual void begin(ftl::rgbd::Frame &, ftl::codecs::Channel)=0;

	/**
	 * Finish a render. Post process the output as required, or finish
	 * generating it from internal buffers. The output frame is only valid
	 * after this is called.
	 */
	virtual void end()=0;

	/**
	 * Perform any final render pass that combines all results. This should be
	 * called after all data has been submitted.
	 */
	virtual void render()=0;

	virtual void blend(ftl::codecs::Channel)=0;

	/**
	 * If the render fails part way with an exception, it is necessary to call
	 * this method to reset the renderer before trying the next frame.
	 */
	virtual void cancel()=0;

	protected:
	Stage stage_;
};

/**
 * A renderer specifically for RGB-D framesets. To use an instance, call
 * `begin`, `submit` (multiple), `render` and `end` methods in that order. Note
 * that usually the `submit` and `render` calls initiate asynchronous jobs on
 * the GPU and so don't block. Only `end` will block until finished. The GPU
 * stream used, if any, is the output frame's CUDA stream.
 */
class FSRenderer : public ftl::render::Renderer {
	public:
	explicit FSRenderer(nlohmann::json &config) : ftl::render::Renderer(config) {};
	virtual ~FSRenderer() {};

	/**
     * Render all frames of a frameset into the output frame. This can be called
	 * multiple times between `begin` and `render` to combine multiple framesets.
	 * Note that the frameset pointer must remain valid until `end` is called,
	 * and ideally should not be swapped between.
	 *
	 * The channels parameter gives all of the source channels that will be
	 * rendered into the single colour output. These will be blended
	 * together by some undefined method. Non colour channels will be converted
	 * to RGB colour appropriately. Config options may be able to control this
	 * process. The final matrix is an addition transform to apply to this
	 * frameset before rendering, it can be identity.
     */
    virtual bool submit(ftl::data::FrameSet *, ftl::codecs::Channels<0>, const Eigen::Matrix4d &)=0;
};

}
}

#endif  // _FTL_RENDER_RENDERER_HPP_
