#ifndef _FTL_OPERATORS_SMOOTHING_HPP_
#define _FTL_OPERATORS_SMOOTHING_HPP_

#include <ftl/operators/operator.hpp>

namespace ftl {
namespace operators {

/**
 * Remove high frequency noise whilst attempting to retain sharp edges and curves.
 * It uses the minimum error in the second derivative of the local surface as
 * a smoothing factor, with the depth error itself taken from the minimum
 * first derivative. Only the depth channel is used and modified.
 */
class HFSmoother : public ftl::operators::Operator {
    public:
    explicit HFSmoother(ftl::Configurable*);
    ~HFSmoother();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

    private:
    cv::cuda::GpuMat temp_;
    ftl::rgbd::Frame frames_[4];
};

/**
 * Generate a smoothing channel from the colour image that provides a smoothing
 * factor for each pixel. It uses colour gradient at multiple resolutions to
 * decide on how much a given pixel needs smoothing, large single colour areas
 * will generate a large smoothing value, whilst sharp colour edges will have
 * no smoothing.
 */
class SmoothChannel : public ftl::operators::Operator {
    public:
    explicit SmoothChannel(ftl::Configurable*);
    ~SmoothChannel();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

	private:
	ftl::rgbd::Frame temp_[6];
};

/**
 * Perform Moving Least Squares smoothing with a constant smoothing amount and
 * neighbourhood size. Requires channels: Depth + Normals.
 * Also outputs Depth + Normals.
 */
class SimpleMLS : public ftl::operators::Operator {
    public:
    explicit SimpleMLS(ftl::Configurable*);
    ~SimpleMLS();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

    private:
};

/**
 * Perform Moving Least Squares smoothing with a smoothing amount determined
 * by a simple colour similarity weighting. In practice this is too naive.
 */
class ColourMLS : public ftl::operators::Operator {
    public:
    explicit ColourMLS(ftl::Configurable*);
    ~ColourMLS();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

    private:
};

/**
 * A version of Moving Least Squares where both the smoothing factor and
 * neighbourhood size are adapted over an extra large range using the colour
 * channel as a guide. Requires Depth+Normals+Colour. Neighbourhood can
 * encompass hundreds of pixels with a smoothing factor approaching a meter, or
 * it can be only a few or even no pixels with a zero smoothing factor.
 */
class AdaptiveMLS : public ftl::operators::Operator {
    public:
    explicit AdaptiveMLS(ftl::Configurable*);
    ~AdaptiveMLS();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

    bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, ftl::rgbd::Source *src, cudaStream_t stream) override;

    private:
};

}
}

#endif  // _FTL_OPERATORS_SMOOTHING_HPP_
