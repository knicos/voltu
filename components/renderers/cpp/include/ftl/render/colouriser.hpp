#ifndef _FTL_RENDER_COLOURISER_HPP_
#define _FTL_RENDER_COLOURISER_HPP_

#include <ftl/configurable.hpp>
#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/frame.hpp>
#include <ftl/codecs/channels.hpp>

namespace ftl {
namespace render {

uchar4 parseCUDAColour(const std::string &colour);
cv::Scalar parseCVColour(const std::string &colour);

/**
 * Generate a colour texture for any frame channel. It also can modify the
 * existing colour channels. All settings come from the json config.
 */
class Colouriser : public ftl::Configurable {
	public:
	explicit Colouriser(nlohmann::json &config);
	~Colouriser();

	ftl::cuda::TextureObject<uchar4> &colourise(ftl::rgbd::Frame &f, ftl::codecs::Channel, cudaStream_t stream);

	private:
	std::list<ftl::cuda::TextureObject<uchar4>*> textures_;
	cv::cuda::GpuMat depth_gray_;
	cv::cuda::GpuMat depth_bgr_;

	ftl::cuda::TextureObject<uchar4> &_getBuffer(size_t width, size_t height);

	template <typename T>
	ftl::cuda::TextureObject<uchar4> &_processSingle(ftl::rgbd::Frame &f, ftl::codecs::Channel c, cudaStream_t stream);

	ftl::cuda::TextureObject<uchar4> &_processColour(ftl::rgbd::Frame &f, ftl::codecs::Channel c, cudaStream_t stream);
	ftl::cuda::TextureObject<uchar4> &_processNormals(ftl::rgbd::Frame &f, ftl::codecs::Channel c, cudaStream_t stream);
	ftl::cuda::TextureObject<uchar4> &_processFloat(ftl::rgbd::Frame &f, ftl::codecs::Channel c, float minval, float maxval, cudaStream_t stream);
};

}
}

#endif