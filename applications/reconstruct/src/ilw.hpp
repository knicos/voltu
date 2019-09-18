#ifndef _FTL_RECONSTRUCT_ILW_HPP_
#define _FTL_RECONSTRUCT_ILW_HPP_

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/configurable.hpp>
#include <vector>

namespace ftl {

namespace detail {
struct ILWData{
    // x,y,z + confidence
    ftl::cuda::TextureObject<float4> correspondence;

    ftl::cuda::TextureObject<float4> points;

	// Residual potential energy
	ftl::cuda::TextureObject<float> residual;

	// Flow magnitude
	ftl::cuda::TextureObject<float> flow;
};
}

/**
 * For a set of sources, perform Iterative Lattice Warping to correct the
 * location of points between the cameras. The algorithm finds possible
 * correspondences and warps the original pixel lattice of points in each
 * camera towards the correspondences, iterating the process as many times as
 * possible. The result is that both local and global adjustment is made to the
 * point clouds to improve micro alignment that may have been incorrect due to
 * either inaccurate camera pose estimation or noise/errors in the depth maps.
 */
class ILW : public ftl::Configurable {
    public:
    explicit ILW(nlohmann::json &config);
    ~ILW();

    /**
     * Take a frameset and perform the iterative lattice warping.
     */
    bool process(ftl::rgbd::FrameSet &fs);

    private:
    /*
     * Initialise data.
     */
    bool _phase0(ftl::rgbd::FrameSet &fs);

    /*
     * Find possible correspondences and a confidence value.
     */
    bool _phase1(ftl::rgbd::FrameSet &fs);

    /*
     * Calculate energies and move the points.
     */
    bool _phase2(ftl::rgbd::FrameSet &fs);

    std::vector<detail::ILWData> data_;
};

}

#endif  // _FTL_RECONSTRUCT_ILW_HPP_
