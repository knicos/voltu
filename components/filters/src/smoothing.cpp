#include <ftl/filters/smoothing.hpp>
#include "smoothing_cuda.hpp"

using ftl::DepthSmoother;
using ftl::codecs::Channel;
using cv::cuda::GpuMat;

DepthSmoother::DepthSmoother(nlohmann::json &config) : ftl::Configurable(config) {

}

DepthSmoother::~DepthSmoother() {

}

void DepthSmoother::smooth(ftl::rgbd::Frame &f, ftl::rgbd::Source *s) {
    float var_thresh = value("variance_threshold", 0.0002f);
    bool do_smooth = value("pre_smooth", false);
    int levels = max(0, min(value("levels",0), 4));
    int iters = value("iterations",5);

    if (!do_smooth) return;

    for (int i=0; i<iters; ++i) {
        ftl::cuda::smoothing_factor(
            f.createTexture<float>(Channel::Depth),
            f.createTexture<float>(Channel::Energy, ftl::rgbd::Format<float>(f.get<cv::cuda::GpuMat>(Channel::Depth).size())),
            f.createTexture<float>(Channel::Smoothing, ftl::rgbd::Format<float>(f.get<cv::cuda::GpuMat>(Channel::Depth).size())),
            var_thresh,
            s->parameters(), 0
        );
    }

    LOG(INFO) << "PARAMS DEPTHS  " << s->parameters().minDepth << "," << s->parameters().maxDepth;

    for (int i=0; i<levels; ++i) {
        var_thresh *= 2.0f;
        auto &dmat = f.get<GpuMat>(Channel::Depth); 
        cv::cuda::resize(dmat, frames_[i].create<GpuMat>(Channel::Depth), cv::Size(dmat.cols / (2*(i+1)), dmat.rows / (2*(i+1))), 0.0, 0.0, cv::INTER_NEAREST);

        ftl::cuda::smoothing_factor(
            frames_[i].createTexture<float>(Channel::Depth),
            frames_[i].createTexture<float>(Channel::Energy, ftl::rgbd::Format<float>(frames_[i].get<GpuMat>(Channel::Depth).size())),
            frames_[i].createTexture<float>(Channel::Smoothing, ftl::rgbd::Format<float>(frames_[i].get<GpuMat>(Channel::Depth).size())),
            var_thresh,
            s->parameters(), 0
        );

        cv::cuda::resize(frames_[i].get<GpuMat>(Channel::Smoothing), temp_, f.get<cv::cuda::GpuMat>(Channel::Depth).size(), 0.0, 0.0, cv::INTER_LINEAR);
        cv::cuda::add(temp_, f.get<GpuMat>(Channel::Smoothing), f.get<GpuMat>(Channel::Smoothing));
    }

    //cv::cuda::subtract(f.get<GpuMat>(Channel::Depth), f.get<GpuMat>(Channel::Smoothing), f.get<GpuMat>(Channel::Depth));
}

