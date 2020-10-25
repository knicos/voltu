#ifndef _FTL_RENDER_GL_HPP_
#define _FTL_RENDER_GL_HPP_

#include <ftl/render/renderer.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/render/render_params.hpp>
#include <ftl/codecs/channels.hpp>
#include <ftl/utility/gltexture.hpp>

// Forward declare
namespace nanogui {
class GLShader;
}

namespace ftl {
namespace render {

class Colouriser;

class GLRender : public ftl::render::FSRenderer {
	public:
	explicit GLRender(nlohmann::json &config);
	~GLRender();

	void begin(ftl::rgbd::Frame &, ftl::codecs::Channel) override;
	void end() override;

	bool submit(ftl::data::FrameSet *in, ftl::codecs::Channels<0>, const Eigen::Matrix4d &t) override;

	void render() override;

	void blend(ftl::codecs::Channel) override;

	void cancel() override;

	private:
	ftl::rgbd::Frame *out_=nullptr;

	ftl::render::Colouriser *colouriser_;

	cv::Scalar background_;
	float3 light_dir_;
	uchar4 light_diffuse_;
	uchar4 light_ambient_;
	ftl::render::Parameters params_;
	float3 light_pos_;

	ftl::codecs::Channel out_chan_;

	struct SubmitState {
		ftl::rgbd::FrameSet *fs;
		ftl::codecs::Channels<0> channels;
		Eigen::Matrix4d transform;
	};

	std::vector<SubmitState> sets_;

	nanogui::GLShader *shader_;
	std::vector<float> vertices_;
	std::vector<int> indices_;
	unsigned int outTex_=0;
	unsigned int outPBO_=0;
	unsigned int fbuf_=0;
	unsigned int depthbuf_=0;
	cudaGraphicsResource *cuda_res_=nullptr;

	//std::unordered_map<unsigned int, ftl::utility::GLTexture> textures_;
	ftl::utility::GLTexture depth_input_;
	ftl::utility::GLTexture colour_input_;
};

}
}

#endif  // _FTL_RENDER_GL_HPP_
