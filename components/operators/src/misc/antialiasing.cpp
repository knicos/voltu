#include <ftl/operators/antialiasing.hpp>
#include "antialiasing_cuda.hpp"

using ftl::operators::FXAA;
using ftl::codecs::Channel;

FXAA::FXAA(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

FXAA::~FXAA() {

}

bool FXAA::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	if (in.hasChannel(Channel::Depth)) {
		ftl::cuda::fxaa(
			in.getTexture<uchar4>(Channel::Colour),
			in.getTexture<float>(Channel::Depth),
			config()->value("threshold", 0.1f),
			stream
		);
	} else {
		ftl::cuda::fxaa(
			in.getTexture<uchar4>(Channel::Colour),
			stream
		);
	}

	if (in.hasChannel(Channel::Right)) {
		ftl::cuda::fxaa(
			in.getTexture<uchar4>(Channel::Right),
			stream
		);
	}

	return true;
}
