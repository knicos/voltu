#include "player.hpp"
#include <ftl/configuration.hpp>

using ftl::rgbd::Player;

Player::Player(std::istream &s) : stream_(&s), reader_(s) {
    auto *c = ftl::config::find("/controls");

    offset_ = 0;
	
    if (c) {
        paused_ = c->value("paused", false);
        c->on("paused", [this,c](const ftl::config::Event &e) {
            UNIQUE_LOCK(mtx_, lk);
            paused_ = c->value("paused", false);
            if (paused_) {
                pause_time_ = last_ts_;
            } else {
                offset_ += last_ts_ - pause_time_;
            }
        });

        looping_ = c->value("looping", true);
        c->on("looping", [this,c](const ftl::config::Event &e) {
            looping_ = c->value("looping", false);
        });

        speed_ = c->value("speed", 1.0f);
        c->on("speed", [this,c](const ftl::config::Event &e) {
            speed_ = c->value("speed", 1.0f);
        });

        reversed_ = c->value("reverse", false);
        c->on("reverse", [this,c](const ftl::config::Event &e) {
            reversed_ = c->value("reverse", false);
        });
    }
}

Player::~Player() {
    // TODO: Remove callbacks
}

bool Player::read(int64_t ts) {
    std::unique_lock<std::mutex> lk(mtx_, std::defer_lock);
	if (!lk.try_lock()) return true;

    last_ts_ = ts;
    if (paused_) return true;

    int64_t adjusted_ts = int64_t(float(ts - reader_.getStartTime()) * speed_) + reader_.getStartTime() + offset_;
    bool res = reader_.read(adjusted_ts);

    if (looping_ && !res) {
        reader_.end();
        offset_ = 0;
        stream_->clear();
        stream_->seekg(0);
        if (!reader_.begin()) {
            LOG(ERROR) << "Could not loop";
            return false;
        }
        return true;
    }

    return res;
}

void Player::onPacket(int streamID, const std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)> &f) {
    reader_.onPacket(streamID, f);
}

bool Player::begin() {
    return reader_.begin();
}

bool Player::end() {
    return reader_.end();
}
