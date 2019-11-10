#include <ftl/operators/filling.hpp>

#include "filling_cuda.hpp"

using ftl::operators::ScanFieldFill;
using ftl::operators::CrossSupportFill;
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


CrossSupportFill::CrossSupportFill(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

CrossSupportFill::~CrossSupportFill() {

}

bool CrossSupportFill::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *s, cudaStream_t stream) {

	/*ftl::cuda::filling_csr(
		in.createTexture<uchar4>(Channel::Colour2),
		in.createTexture<float4>(Channel::Normals),
		in.createTexture<float>(Channel::Depth),
		in.createTexture<float>(Channel::Depth),
		in.createTexture<uchar4>(Channel::Colour),
		s->parameters(), 0
	);*/

	return true;
}
