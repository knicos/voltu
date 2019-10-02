#ifndef _FTL_RECONSTRUCTION_SPLAT_HPP_
#define _FTL_RECONSTRUCTION_SPLAT_HPP_

#include <ftl/render/renderer.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/render/splat_params.hpp>
#include <ftl/cuda/points.hpp>

namespace ftl {
namespace render {

/**
 * Render the voxel hash structure by generating image points for surface
 * voxels and expanding those into interpolated splats. This is a two pass
 * algorithm with the option of completing the second pass on a separate GPU.
 * It also possible to only complete the first pass and perform the second step
 * on a separate machine or at a later time, the advantage being to save local
 * processing resources and that the first pass result may compress better.
 */
class Splatter : public ftl::render::Renderer {
	public:
	explicit Splatter(nlohmann::json &config, ftl::rgbd::FrameSet *fs);
	~Splatter();

	bool render(ftl::rgbd::VirtualSource *src, ftl::rgbd::Frame &out, cudaStream_t stream=0) override;
	//void setOutputDevice(int);

	protected:
	void renderChannel(ftl::render::SplatParams &params, ftl::rgbd::Frame &out, const ftl::rgbd::Channel &channel, cudaStream_t stream);

	private:
	int device_;
	/*ftl::cuda::TextureObject<int> depth1_;
	ftl::cuda::TextureObject<int> depth3_;
	ftl::cuda::TextureObject<uchar4> colour1_;
	ftl::cuda::TextureObject<float4> colour_tmp_;
	ftl::cuda::TextureObject<float> depth2_;
	ftl::cuda::TextureObject<uchar4> colour2_;
	ftl::cuda::TextureObject<float4> normal1_;*/
	//SplatParams params_;

	ftl::rgbd::Frame temp_;
	ftl::rgbd::FrameSet *scene_;
	ftl::cuda::ClipSpace clip_;
	bool clipping_;
	float norm_filter_;
};

}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_HPP_
