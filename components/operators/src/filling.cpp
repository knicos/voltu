#include <ftl/operators/filling.hpp>

#include "filling_cuda.hpp"

using ftl::operators::ScanFieldFill;
using ftl::codecs::Channel;

ScanFieldFill::ScanFieldFill(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

ScanFieldFill::~ScanFieldFill() {

}

bool ScanFieldFill::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {
	float thresh = config()->value("threshold", 0.1f);

	ftl::cuda::scan_field_fill(
		in.createTexture<float>(Channel::Depth),
		in.createTexture<float>(Channel::Depth),
		in.createTexture<float>(Channel::Smoothing),
		thresh,
		s->parameters(), 0
	);

	return true;
}
