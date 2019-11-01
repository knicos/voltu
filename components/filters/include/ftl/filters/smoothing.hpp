#ifndef _FTL_SMOOTHING_HPP_
#define _FTL_SMOOTHING_HPP_

#include <ftl/configurable.hpp>
#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/frame.hpp>

namespace ftl {

class DepthSmoother : public ftl::Configurable {
    public:
    explicit DepthSmoother(nlohmann::json &config);
    ~DepthSmoother();

    void smooth(ftl::rgbd::Frame &frame, ftl::rgbd::Source *src);

    private:
    cv::cuda::GpuMat temp_;
    ftl::rgbd::Frame frames_[4];
};

}

#endif  // _FTL_SMOOTHING_HPP_
