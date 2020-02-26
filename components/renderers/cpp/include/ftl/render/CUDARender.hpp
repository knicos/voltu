#ifndef _FTL_RENDER_CUDA_HPP_
#define _FTL_RENDER_CUDA_HPP_

#include <ftl/render/renderer.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/render/render_params.hpp>
#include <ftl/cuda/points.hpp>
#include <ftl/codecs/channels.hpp>
//#include <ftl/filters/filter.hpp>

namespace ftl {
namespace render {

class Colouriser;

/**
 * Generate triangles between connected points and render those. Colour is done
 * by weighted reprojection to the original source images.
 */
class CUDARender : public ftl::render::Renderer {
	public:
	explicit CUDARender(nlohmann::json &config);
	~CUDARender();

	void begin(ftl::rgbd::Frame &, ftl::codecs::Channel) override;
	void end() override;

	bool submit(ftl::rgbd::FrameSet *in, ftl::codecs::Channels<0>, const Eigen::Matrix4d &t) override;
	//void setOutputDevice(int);

	void blend(ftl::codecs::Channel) override;

	void setViewPort(ftl::render::ViewPortMode mode, const ftl::render::ViewPort &vp) {
		params_.viewport = vp;
		params_.viewPortMode = mode;
	}

	cudaStream_t getCUDAStream() const { return stream_; }

	protected:
	void _renderChannel(ftl::rgbd::Frame &out, ftl::codecs::Channel channel_in, const Eigen::Matrix4d &t, cudaStream_t stream);

	private:
	int device_;
	ftl::rgbd::Frame temp_;
	//ftl::rgbd::Frame accum_;
	ftl::cuda::TextureObject<float4> accum_;		// 2 is number of channels can render together
	ftl::cuda::TextureObject<int> contrib_;
	//ftl::cuda::TextureObject<half4> normals_;

	std::list<ftl::cuda::TextureObject<short2>*> screen_buffers_;
	std::list<ftl::cuda::TextureObject<float>*> depth_buffers_;

	ftl::rgbd::Frame *out_;
	ftl::rgbd::FrameSet *scene_;
	ftl::cuda::ClipSpace clip_;
	ftl::render::Colouriser *colouriser_;
	bool clipping_;
	float norm_filter_;
	bool backcull_;
	cv::Scalar background_;
	bool mesh_;
	float3 light_dir_;
	uchar4 light_diffuse_;
	uchar4 light_ambient_;
	ftl::render::Parameters params_;
	cudaStream_t stream_;
	float3 light_pos_;
	Eigen::Matrix4d transform_;
	float4x4 pose_;
	float4x4 poseInverse_;
	float scale_;
	int64_t last_frame_;
	ftl::codecs::Channel out_chan_;

	cv::cuda::GpuMat env_image_;
	ftl::cuda::TextureObject<uchar4> env_tex_;

	//ftl::Filters *filters_;

	struct SubmitState {
		ftl::rgbd::FrameSet *fs;
		ftl::codecs::Channels<0> channels;
		Eigen::Matrix4d transform;
	};

	std::vector<SubmitState> sets_;

	void _dibr(ftl::rgbd::Frame &, const Eigen::Matrix4d &t, cudaStream_t);
	void _mesh(ftl::rgbd::Frame &, const Eigen::Matrix4d &t, cudaStream_t);
	void _updateParameters(ftl::rgbd::Frame &out, ftl::codecs::Channel);
	void _allocateChannels(ftl::rgbd::Frame &out, ftl::codecs::Channel);
	void _postprocessColours(ftl::rgbd::Frame &out);

	void _renderPass1(const Eigen::Matrix4d &t);
	void _renderPass2(ftl::codecs::Channels<0>, const Eigen::Matrix4d &t);

	void _end();
	void _endSubmit();

	bool _alreadySeen() const { return last_frame_ == scene_->timestamp; }
	void _adjustDepthThresholds(const ftl::rgbd::Camera &fcam);

	ftl::cuda::TextureObject<float> &_getDepthBuffer(const cv::Size &);
	ftl::cuda::TextureObject<short2> &_getScreenBuffer(const cv::Size &);

	inline ftl::codecs::Channel _getDepthChannel() const { return (out_chan_ == ftl::codecs::Channel::Colour) ? ftl::codecs::Channel::Depth : ftl::codecs::Channel::Depth2; }
	inline ftl::codecs::Channel _getNormalsChannel() const { return (out_chan_ == ftl::codecs::Channel::Colour) ? ftl::codecs::Channel::Normals : ftl::codecs::Channel::Normals2; }
};

}
}

#endif  // _FTL_RENDER_CUDA_HPP_
