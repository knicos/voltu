#ifndef _FTL_RENDER_ASSIMP_HPP_
#define _FTL_RENDER_ASSIMP_HPP_

#include <ftl/render/renderer.hpp>
#include <ftl/render/assimp_scene.hpp>
//#include <GL/gl.h>
//#include <GL/glext.h>
#include <nanogui/glutil.h>

namespace ftl {
namespace render {

/**
 * Render Assimp library models using OpenGL into a frame object. The
 * channels will be OpenGL pixel buffer objects, or should be created as such
 * before hand. Begin, end, render etc must also be called in a valid OpenGL
 * context.
 */
class AssimpRenderer : public ftl::render::Renderer {
	public:
	explicit AssimpRenderer(nlohmann::json &config);
	~AssimpRenderer();

	void begin(ftl::rgbd::Frame &, ftl::codecs::Channel) override;

	void end() override;

	void render() override;

	void blend(ftl::codecs::Channel) override;

	void setScene(ftl::render::AssimpScene *);

	/**
	 * Set the pose / model view matix for the scene not the camera.
	 */
	void setPose(const Eigen::Matrix4d &pose);

	private:
	AssimpScene *scene_;
	ftl::rgbd::Frame *out_;
	ftl::codecs::Channel outchan_;
	nanogui::GLShader shader_;
	bool init_;
};

}
}

#endif
