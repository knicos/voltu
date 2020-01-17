#ifndef _FTL_RECONSTRUCTION_TRI_HPP_
#define _FTL_RECONSTRUCTION_TRI_HPP_

#include <ftl/render/renderer.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/render/splat_params.hpp>
#include <ftl/cuda/points.hpp>
//#include <ftl/filters/filter.hpp>

namespace ftl {
namespace render {

/**
 * Generate triangles between connected points and render those. Colour is done
 * by weighted reprojection to the original source images.
 */
class Triangular : public ftl::render::Renderer {
	public:
	explicit Triangular(nlohmann::json &config);
	~Triangular();

	bool render(ftl::rgbd::FrameSet &in, ftl::rgbd::Frame &out, ftl::codecs::Channel, const Eigen::Matrix4d &t) override;
	//void setOutputDevice(int);

	protected:
	void _renderChannel(ftl::rgbd::Frame &out, ftl::codecs::Channel channel_in, ftl::codecs::Channel channel_out, const Eigen::Matrix4d &t, cudaStream_t stream);

	private:
	int device_;
	ftl::rgbd::Frame temp_;
	ftl::rgbd::Frame accum_;
	ftl::rgbd::FrameSet *scene_;
	ftl::cuda::ClipSpace clip_;
	bool clipping_;
	float norm_filter_;
	bool backcull_;
	cv::Scalar background_;
	bool mesh_;
	float3 light_dir_;
	uchar4 light_diffuse_;
	uchar4 light_ambient_;
	ftl::render::SplatParams params_;
	cudaStream_t stream_;
	float3 light_pos_;
	Eigen::Matrix4d transform_;
	float scale_;
	int64_t last_frame_;

	cv::cuda::GpuMat env_image_;
	ftl::cuda::TextureObject<uchar4> env_tex_;

	//ftl::Filters *filters_;

	template <typename T>
	void __reprojectChannel(ftl::rgbd::Frame &, ftl::codecs::Channel in, ftl::codecs::Channel out, const Eigen::Matrix4d &t, cudaStream_t);
	void _reprojectChannel(ftl::rgbd::Frame &, ftl::codecs::Channel in, ftl::codecs::Channel out, const Eigen::Matrix4d &t, cudaStream_t);
	void _dibr(ftl::rgbd::Frame &, const Eigen::Matrix4d &t, cudaStream_t);
	void _mesh(ftl::rgbd::Frame &, const Eigen::Matrix4d &t, cudaStream_t);
	void _preprocessColours();
	void _updateParameters(ftl::rgbd::Frame &out);
	void _allocateChannels(ftl::rgbd::Frame &out);
	void _postprocessColours(ftl::rgbd::Frame &out);

	void _renderNormals(ftl::rgbd::Frame &out);
	void _renderDensity(ftl::rgbd::Frame &out, const Eigen::Matrix4d &t);
	void _renderRight(ftl::rgbd::Frame &out, const Eigen::Matrix4d &t);
	void _renderSecond(ftl::rgbd::Frame &out, ftl::codecs::Channel chan, const Eigen::Matrix4d &t);

	bool _alreadySeen() const { return last_frame_ == scene_->timestamp; }
};

}
}

#endif  // _FTL_RECONSTRUCTION_TRI_HPP_
