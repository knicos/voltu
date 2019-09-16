#ifndef _FTL_RGBD_CHANNELS_HPP_
#define _FTL_RGBD_CHANNELS_HPP_

#include <bitset>
#include <msgpack.hpp>

namespace ftl {
namespace rgbd {

enum struct Channel : int {
    None = -1,
    Colour = 0,
    Left = 0,
    Depth = 1,
    Right,
    Disparity,
    Deviation,
    Normals,
    Confidence,
    Flow,
    Energy,
    LeftGray,
    RightGray,
    Overlay1
};

class Channels {
    public:
    inline Channels() { mask = 0; }
    inline explicit Channels(unsigned int m) { mask = m; }
    inline explicit Channels(Channel c) { mask = (c == Channel::None) ? 0 : 0x1 << static_cast<unsigned int>(c); }
    inline Channels &operator=(Channel c) { mask = (c == Channel::None) ? 0 : 0x1 << static_cast<unsigned int>(c); return *this; }
    inline Channels operator|(Channel c) const { return (c == Channel::None) ? Channels(mask) : Channels(mask | (0x1 << static_cast<unsigned int>(c))); }
    inline Channels operator+(Channel c) const { return (c == Channel::None) ? Channels(mask) : Channels(mask | (0x1 << static_cast<unsigned int>(c))); }
    inline Channels &operator|=(Channel c) { mask |= (c == Channel::None) ? 0 : (0x1 << static_cast<unsigned int>(c)); return *this; }
    inline Channels &operator+=(Channel c) { mask |= (c == Channel::None) ? 0 : (0x1 << static_cast<unsigned int>(c)); return *this; }
    inline Channels &operator-=(Channel c) { mask &= ~((c == Channel::None) ? 0 : (0x1 << static_cast<unsigned int>(c))); return *this; }
    inline Channels &operator+=(unsigned int c) { mask |= (0x1 << c); return *this; }
    inline Channels &operator-=(unsigned int c) { mask &= ~(0x1 << c); return *this; }

    inline bool has(Channel c) const {
        return (c == Channel::None) ? true : mask & (0x1 << static_cast<unsigned int>(c));
    }

    inline bool has(unsigned int c) const {
        return mask & (0x1 << c);
    }

    inline operator unsigned int() { return mask; }
    inline operator bool() { return mask > 0; }
    inline operator Channel() {
        if (mask == 0) return Channel::None;
        int ix = 0;
        int tmask = mask;
        while (!(tmask & 0x1) && ++ix < 32) tmask >>= 1;
        return static_cast<Channel>(ix);
    }
    
    inline size_t count() { return std::bitset<32>(mask).count(); }
    inline void clear() { mask = 0; }

    static const size_t kMax = 32;

    private:
    unsigned int mask;
};

static const Channels kNoChannels;
static const Channels kAllChannels(0xFFFFFFFFu);

inline bool isFloatChannel(ftl::rgbd::Channel chan) {
	switch (chan) {
	case Channel::Depth		:
	case Channel::Energy	: return true;
	default					: return false;
	}
}

}
}

MSGPACK_ADD_ENUM(ftl::rgbd::Channel);

inline ftl::rgbd::Channels operator|(ftl::rgbd::Channel a, ftl::rgbd::Channel b) {
    return ftl::rgbd::Channels(a) | b;
}

inline ftl::rgbd::Channels operator+(ftl::rgbd::Channel a, ftl::rgbd::Channel b) {
    return ftl::rgbd::Channels(a) | b;
}

#endif  // _FTL_RGBD_CHANNELS_HPP_
