#include <ftl/rgbd/detail/abr.hpp>
#include <ftl/timer.hpp>

#include <bitset>

using ftl::rgbd::detail::BitrateSetting;
using ftl::rgbd::detail::ABRController;
using ftl::rgbd::detail::bitrate_t;
using ftl::rgbd::detail::kBitrateWorst;
using ftl::rgbd::detail::kBitrateBest;
using ftl::rgbd::detail::bitrate_settings;
using ftl::rgbd::detail::NetFrame;

ABRController::ABRController() {
    bitrate_ = 0;
    enabled_ = false;
    max_ = kBitrateBest;
    min_ = kBitrateWorst;
}

ABRController::~ABRController() {

}

void ABRController::setMaximumBitrate(bitrate_t b) {
    max_ = (b == -1) ? kBitrateBest : b;
    if (bitrate_ < max_) bitrate_ = max_;
}

void ABRController::setMinimumBitrate(bitrate_t b) {
    min_ = (b == -1) ? kBitrateWorst : b;
    if (bitrate_ > min_) bitrate_ = min_;
}

void ABRController::notifyChanged() {
    //enabled_ = true;
}

bitrate_t ABRController::selectBitrate(const NetFrame &frame) {
    if (!enabled_) return bitrate_;

    float actual_mbps = (float(frame.tx_size) * 8.0f * (1000.0f / float(frame.tx_latency))) / 1048576.0f;
    float min_mbps = (float(frame.tx_size) * 8.0f * (1000.0f / float(ftl::timer::getInterval()))) / 1048576.0f;
    LOG(INFO) << "Bitrate = " << actual_mbps << "Mbps, min required = " << min_mbps << "Mbps";
    float ratio = actual_mbps / min_mbps;
    //LOG(INFO) << "Rate Ratio = " << frame.tx_latency;

    down_log_ = down_log_ << 1;
    up_log_ = up_log_ << 1;

    if (ratio < 1.2f) {
        down_log_ += 1;
    } else if (ratio > 1.5f) {
        up_log_ += 1;
    }

    std::bitset<32> bd(down_log_);
    std::bitset<32> bu(up_log_);

    if (bitrate_ < min_ && int(bd.count()) - int(bu.count()) > 5) {
        enabled_ = false;
        down_log_ = 0;
        up_log_ = 0;
        bitrate_++;
        LOG(INFO) << "Bitrate down to: " << bitrate_;
    } else if (bitrate_ > max_ && int(bu.count()) - int(bd.count()) > 15) {
        enabled_ = false;
        up_log_ = 0;
        down_log_ = 0;
        bitrate_--;
        LOG(INFO) << "Bitrate up to: " << bitrate_;
    }

    return bitrate_;
}

const BitrateSetting &ABRController::getBitrateInfo(bitrate_t b) {
    if (b > kBitrateWorst) return bitrate_settings[kBitrateWorst];
    if (b < kBitrateBest) return bitrate_settings[kBitrateBest];
    return bitrate_settings[b];
};

int ABRController::getColourWidth(bitrate_t b) {
    return std::ceil(bitrate_settings[b].colour_res * kAspectRatio);
}

int ABRController::getDepthWidth(bitrate_t b) {
    return std::ceil(bitrate_settings[b].depth_res * kAspectRatio);
}

int ABRController::getColourHeight(bitrate_t b) {
    return bitrate_settings[b].colour_res;
}

int ABRController::getDepthHeight(bitrate_t b) {
    return bitrate_settings[b].depth_res;
}

int ABRController::getBlockCountX(bitrate_t b) {
    return bitrate_settings[b].block_count_x;
}

int ABRController::getBlockCountY(bitrate_t b) {
    return bitrate_settings[b].block_count_x;
}

int ABRController::getBlockCount(bitrate_t b) {
    const int c = bitrate_settings[b].block_count_x;
    return c*c;
}

int ABRController::getColourQuality(bitrate_t b) {
    return bitrate_settings[b].colour_qual;
}

int ABRController::getDepthQuality(bitrate_t b) {
    return bitrate_settings[b].depth_qual;
}
