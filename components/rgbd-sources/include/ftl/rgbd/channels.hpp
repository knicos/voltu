#ifndef _FTL_RGBD_CHANNELS_HPP_
#define _FTL_RGBD_CHANNELS_HPP_

#include <bitset>
#include <msgpack.hpp>

namespace ftl {
namespace rgbd {

enum struct Channel : int {
    None = -1,
    Colour = 0,         // 8UC3 or 8UC4
    Left = 0,
    Depth = 1,          // 32S or 32F
    Right = 2,          // 8UC3 or 8UC4
    Colour2 = 2,
    Disparity = 3,
    Depth2 = 3,
    Deviation = 4,
    Normals,            // 32FC4
    Points,             // 32FC4
    Confidence,         // 32F
    Flow,               // 32F
    Energy,             // 32F
    LeftGray,
    RightGray,
    Overlay1
};

class Channels {
    public:

	class iterator {
		public:
		iterator(const Channels &c, unsigned int ix) : channels_(c), ix_(ix) { }
		iterator operator++();
		iterator operator++(int junk);
		inline ftl::rgbd::Channel operator*() { return static_cast<Channel>(static_cast<int>(ix_)); }
		//ftl::rgbd::Channel operator->() { return ptr_; }
		inline bool operator==(const iterator& rhs) { return ix_ == rhs.ix_; }
		inline bool operator!=(const iterator& rhs) { return ix_ != rhs.ix_; }
		private:
		const Channels &channels_;
		unsigned int ix_;
	};

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

	inline iterator begin() { return iterator(*this, 0); }
	inline iterator end() { return iterator(*this, 32); }

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

	static Channels All();

    private:
    unsigned int mask;
};

inline Channels::iterator Channels::iterator::operator++() { Channels::iterator i = *this; while (++ix_ < 32 && !channels_.has(ix_)); return i; }
inline Channels::iterator Channels::iterator::operator++(int junk) { while (++ix_ < 32 && !channels_.has(ix_)); return *this; }

inline Channels Channels::All() {
	return Channels(0xFFFFFFFFu);
}

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
