#ifndef _FTL_RGBD_PLAYER_HPP_
#define _FTL_RGBD_PLAYER_HPP_

#include <iostream>
#include <ftl/codecs/reader.hpp>
#include <ftl/threads.hpp>

namespace ftl {
namespace rgbd {

/**
 * Simple wrapper around stream reader to control file playback.
 */
class Player {
    public:
    explicit Player(std::istream &s);
    ~Player();

    bool read(int64_t ts);

	void onPacket(int streamID, const std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)> &);

	bool begin();
	bool end();

    inline int64_t getStartTime() { return reader_.getStartTime(); }

    private:
    std::istream *stream_;
    ftl::codecs::Reader reader_;

    bool paused_;
    bool reversed_;
    bool looping_;
    float speed_;

    int64_t pause_time_;
    int64_t offset_;
    int64_t last_ts_;

    MUTEX mtx_;
};

}
}

#endif  // _FTL_RGBD_PLAYER_HPP_
