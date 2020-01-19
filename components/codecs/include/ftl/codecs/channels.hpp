#ifndef _FTL_RGBD_CHANNELS_HPP_
#define _FTL_RGBD_CHANNELS_HPP_

#include <bitset>
#include <msgpack.hpp>

namespace ftl {
namespace codecs {

enum struct Channel : int {
	None			= -1,
	Colour			= 0,	// 8UC3 or 8UC4
	Left			= 0,
	Depth			= 1,	// 32S or 32F
	Right			= 2,	// 8UC3 or 8UC4
	Colour2			= 2,
	Depth2			= 3,
	Deviation		= 4,
	Screen			= 4,
	Normals			= 5,	// 32FC4
	Points			= 6,	// 32FC4 (should be deprecated)
	Confidence		= 7,	// 32F
	Contribution	= 7,	// 32F
	EnergyVector	= 8,	// 32FC4
	Flow			= 9,	// 32F
	Energy			= 10,	// 32F
	Mask			= 11,	// 32U
	Density			= 12,	// 32F
	Support1		= 13,	// 8UC4 (currently)
	Support2		= 14,	// 8UC4 (currently)
	Segmentation	= 15,	// 32S?
	ColourNormals	= 16,	// 8UC4
	ColourHighRes	= 17,	// 8UC3 or 8UC4
	Disparity		= 18,
	Smoothing		= 19,	// 32F

	AudioLeft		= 32,
	AudioRight		= 33,

	Configuration	= 64,	// JSON Data
	Calibration		= 65,	// Camera Parameters Object
	Pose			= 66,	// Eigen::Matrix4d
	Calibration2	= 67,	// Right camera parameters
	Index           = 68,
	Control			= 69,	// For stream and encoder control
	Data			= 2048	// Custom data, any codec.
};

inline bool isVideo(Channel c) { return (int)c < 32; };
inline bool isAudio(Channel c) { return (int)c >= 32 && (int)c < 64; };
inline bool isData(Channel c) { return (int)c >= 64; };

std::string name(Channel c);
int type(Channel c);

template <int BASE=0>
class Channels {
	public:

	class iterator {
		public:
		iterator(const Channels &c, unsigned int ix) : channels_(c), ix_(ix) { while (ix_ < 32+BASE && !channels_.has(ix_)) ++ix_; }
		inline iterator operator++() { Channels::iterator i = *this; while (++ix_ < 32+BASE && !channels_.has(ix_)); return i; }
		inline iterator operator++(int junk) { while (++ix_ < 32+BASE && !channels_.has(ix_)); return *this; }
		inline ftl::codecs::Channel operator*() { return static_cast<Channel>(static_cast<int>(ix_)); }
		//ftl::codecs::Channel operator->() { return ptr_; }
		inline bool operator==(const iterator& rhs) { return ix_ == rhs.ix_; }
		inline bool operator!=(const iterator& rhs) { return ix_ != rhs.ix_; }
		private:
		const Channels &channels_;
		unsigned int ix_;
	};

	inline Channels() : mask(0) { }
	inline explicit Channels(unsigned int m) : mask(m) { }
	inline explicit Channels(Channel c) : mask((c == Channel::None || static_cast<unsigned int>(c) - BASE >= 32) ? 0 : 0x1 << (static_cast<unsigned int>(c) - BASE)) { }
	inline Channels(const Channels<BASE> &c) : mask((unsigned int)c.mask) {}
	//inline Channels(Channels<BASE> &c) : mask((unsigned int)c.mask) {}
	inline Channels &operator=(const Channels &c) { mask = (unsigned int)c.mask; return *this; }
	inline Channels &operator=(Channel c) { mask = (c == Channel::None || static_cast<unsigned int>(c) - BASE >= 32) ? 0 : 0x1 << (static_cast<unsigned int>(c) - BASE); return *this; }
	inline Channels operator|(Channel c) const { return (c == Channel::None || static_cast<unsigned int>(c) - BASE >= 32) ? Channels(mask) : Channels(mask | (0x1 << (static_cast<unsigned int>(c) - BASE))); }
	inline Channels operator+(Channel c) const { return (c == Channel::None || static_cast<unsigned int>(c) - BASE >= 32) ? Channels(mask) : Channels(mask | (0x1 << (static_cast<unsigned int>(c) - BASE))); }
	inline Channels &operator|=(Channel c) { mask |= (c == Channel::None || static_cast<unsigned int>(c) - BASE >= 32) ? 0 : (0x1 << (static_cast<unsigned int>(c) - BASE)); return *this; }
	inline Channels &operator+=(Channel c) { mask |= (c == Channel::None || static_cast<unsigned int>(c) - BASE >= 32) ? 0 : (0x1 << (static_cast<unsigned int>(c) - BASE)); return *this; }
	inline Channels &operator-=(Channel c) { mask &= ~((c == Channel::None || static_cast<unsigned int>(c) - BASE >= 32) ? 0 : (0x1 << (static_cast<unsigned int>(c) - BASE))); return *this; }
	inline Channels &operator+=(unsigned int c) { mask |= (0x1 << (c - BASE)); return *this; }
	inline Channels &operator-=(unsigned int c) { mask &= ~(0x1 << (c - BASE)); return *this; }
	inline Channels &operator&=(const Channels<BASE> &c) { mask &= c.mask; return *this; }
	inline Channels operator&(const Channels<BASE> &c) const { return Channels<BASE>(mask & c.mask); }
	inline Channels operator-(const Channels<BASE> &c) const { return Channels<BASE>(mask & ~c.mask);}

	inline bool has(Channel c) const {
		return (c == Channel::None || static_cast<unsigned int>(c) - BASE >= 32) ? true : mask & (0x1 << (static_cast<unsigned int>(c) - BASE));
	}

	inline bool has(unsigned int c) const {
		return mask & (0x1 << (c - BASE));
	}

	inline iterator begin() { return iterator(*this, BASE); }
	inline iterator end() { return iterator(*this, 32+BASE); }

	inline bool operator==(const Channels<BASE> &c) const { return mask == c.mask; }
	inline bool operator!=(const Channels<BASE> &c) const { return mask != c.mask; }
	inline operator unsigned int() const { return mask; }
	inline operator bool() const { return mask > 0; }
	inline operator Channel() const {
		if (mask == 0) return Channel::None;
		int ix = 0;
		int tmask = mask;
		while (!(tmask & 0x1) && ++ix < 32) tmask >>= 1;
		return static_cast<Channel>(ix + BASE);
	}
	
	inline size_t count() const { return std::bitset<32>(mask).count(); }
	inline bool empty() const { return mask == 0; }
	inline void clear() { mask = 0; }

	static const size_t kMax = 32;

	static Channels All();

	private:
	std::atomic<unsigned int> mask;
};

template <int BASE>
inline Channels<BASE> Channels<BASE>::All() {
	return Channels<BASE>(0xFFFFFFFFu);
}

static const Channels kNoChannels;
static const Channels kAllChannels(0xFFFFFFFFu);

inline bool isFloatChannel(ftl::codecs::Channel chan) {
	switch (chan) {
	case Channel::Depth		:
	//case Channel::Normals   :
	case Channel::Confidence:
	case Channel::Flow      :
	case Channel::Density:
	case Channel::Energy	: return true;
	default					: return false;
	}
}

}
}

MSGPACK_ADD_ENUM(ftl::codecs::Channel);

template <int BASE=0>
inline ftl::codecs::Channels<BASE> operator|(ftl::codecs::Channel a, ftl::codecs::Channel b) {
	return ftl::codecs::Channels<BASE>(a) | b;
}

template <int BASE=0>
inline ftl::codecs::Channels<BASE> operator+(ftl::codecs::Channel a, ftl::codecs::Channel b) {
	return ftl::codecs::Channels<BASE>(a) | b;
}

#endif  // _FTL_RGBD_CHANNELS_HPP_
