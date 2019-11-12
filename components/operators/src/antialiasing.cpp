#include <ftl/operators/antialiasing.hpp>
#include "antialiasing_cuda.hpp"

using ftl::operators::FXAA;
using ftl::codecs::Channel;

FXAA::FXAA(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

FXAA::~FXAA() {

}

bool FXAA::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	ftl::cuda::fxaa(
		in.getTexture<uchar4>(Channel::Colour),
		stream
	);

	return true;
}
