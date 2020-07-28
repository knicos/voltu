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

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	cv::cuda::GpuMat temp_;
	//ftl::rgbd::Frame frames_[4];
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

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	ftl::cuda::TextureObject<uchar4> temp_[6];
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

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	ftl::data::Frame temp_;
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

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	ftl::data::Frame temp_;
};

/**
 * Use cross aggregation of neighbors for faster MLS smoothing. Utilises the
 * cross support regions and colour weighting. This algorithm is based upon the
 * cross aggregation method in the papers below. MLS was adapted into an
 * approximate form which is not identical to real MLS as the weights are not
 * perfectly correct for each point. The errors are visually minimal and it
 * has linear performance wrt radius, rather than quadratic.
 * 
 * Zhang K, Lu J, Lafruit G. Cross-based local stereo matching using orthogonal
 * integral images. (2009).
 * 
 * Better explained in:
 * X. Mei, X. Sun, M. Zhou et al. On building an accurate stereo matching system
 * on graphics hardware. (2011).
 * 
 * The use of colour weighting is done as in:
 * C. Kuster et al. Spatio-temporal geometry fusion for multiple hybrid cameras
 * using moving least squares surfaces. (2014)
 * 
 * The above paper also indicates the importance of stopping MLS at depth
 * boundaries. They use k-means clustering (K=2) of depths, but we are using
 * an approach (discontinuity mask) indicated in the following paper combined
 * with the cross support regions to define the extent of the MLS neighbourhood.
 * 
 * S. Orts-Escolano et al. Holoportation: Virtual 3D teleportation in real-time.
 * (2016).
 * 
 * The use of all these approaches in this combination is novel I believe.
 * 
 */
class AggreMLS : public ftl::operators::Operator {
	public:
	explicit AggreMLS(ftl::Configurable*);
	~AggreMLS();

	inline Operator::Type type() const override { return Operator::Type::OneToOne; }

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	ftl::cuda::TextureObject<float4> centroid_horiz_;
	ftl::cuda::TextureObject<float4> centroid_vert_;
	ftl::cuda::TextureObject<half4> normals_horiz_;

	ftl::data::Frame temp_;
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

	bool apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) override;

	private:
	ftl::data::Frame temp_;
};

}
}

#endif  // _FTL_OPERATORS_SMOOTHING_HPP_
