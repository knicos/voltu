#ifndef _FTL_OPERATORS_COLOURS_HPP_
#define _FTL_OPERATORS_COLOURS_HPP_

#include <ftl/operators/operator.hpp>

namespace ftl {
namespace operators {

class ColourChannels : public ftl::operators::Operator {
    public:
    explicit ColourChannels(ftl::Configurable *cfg);
    ~ColourChannels();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

    private:
    cv::cuda::GpuMat temp_;
	cv::cuda::GpuMat rbuf_;
};

}
}

#endif // _FTL_OPERATORS_COLOURS_HPP_
