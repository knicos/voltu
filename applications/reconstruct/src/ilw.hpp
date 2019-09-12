#ifndef _FTL_RECONSTRUCT_ILW_HPP_
#define _FTL_RECONSTRUCT_ILW_HPP_

namespace ftl {

/**
 * For a set of sources, perform Iterative Lattice Warping to correct the
 * location of points between the cameras. The algorithm finds possible
 * correspondences and warps the original pixel lattice of points in each
 * camera towards the correspondences, iterating the process as many times as
 * possible. The result is that both local and global adjustment is made to the
 * point clouds to improve micro alignment that may have been incorrect due to
 * either inaccurate camera pose estimation or noise/errors in the depth maps.
 */
class ILW {
    public:
    ILW();
    ~ILW();

    /**
     * Set a physical scene that is composed of a set of source cameras.
     */
    void setScene(ftl::CameraSetScene *);

    /**
     * Take a frameset and perform the iterative lattice warping to update
     * a scene object.
     */
    bool process(ftl::rgbd::FrameSet &fs);

    private:
    ftl::CameraSetScene *scene_;
};

}

#endif  // _FTL_RECONSTRUCT_ILW_HPP_
