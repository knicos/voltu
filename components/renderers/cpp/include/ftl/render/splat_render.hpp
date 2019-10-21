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

	bool render(ftl::rgbd::VirtualSource *src, ftl::rgbd::Frame &out) override;
	//void setOutputDevice(int);

	protected:
	void _renderChannel(ftl::rgbd::Frame &out, ftl::codecs::Channel channel_in, ftl::codecs::Channel channel_out, cudaStream_t stream);

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
	ftl::rgbd::Frame accum_;
	ftl::rgbd::FrameSet *scene_;
	ftl::cuda::ClipSpace clip_;
	bool clipping_;
	float norm_filter_;
	bool backcull_;
	cv::Scalar background_;
	bool splat_;
	float3 light_dir_;
	uchar4 light_diffuse_;
	uchar4 light_ambient_;
	ftl::render::SplatParams params_;
	cudaStream_t stream_;
	float3 light_pos_;

	template <typename T>
	void __blendChannel(ftl::rgbd::Frame &, ftl::codecs::Channel in, ftl::codecs::Channel out, cudaStream_t);
	void _blendChannel(ftl::rgbd::Frame &, ftl::codecs::Channel in, ftl::codecs::Channel out, cudaStream_t);
	void _dibr(cudaStream_t);
};

}
}

#endif  // _FTL_RECONSTRUCTION_SPLAT_HPP_
