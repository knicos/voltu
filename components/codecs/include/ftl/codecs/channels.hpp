/**
 * @file channels.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_RGBD_CHANNELS_HPP_
#define _FTL_RGBD_CHANNELS_HPP_

#include <bitset>
#include <msgpack.hpp>

namespace ftl {
namespace codecs {

// TODO: (nick) Move Channel enum to data component.

/** Frame channel identifier. */
enum struct Channel : int {
	/* Video Channels */
	None			= -1,
	Colour			= 0,	// 8UC3 or 8UC4
	Left			= 0,
	Depth			= 1,	// 32S or 32F
	Right			= 2,	// 8UC3 or 8UC4
	Colour2			= 2,
	Depth2			= 3,
	Deviation		= 4,
	Screen			= 4,	// 16SC2
	Normals			= 5,	// 16FC4
	Weights			= 6,	// short
	Confidence		= 7,	// 32F
	Contribution	= 7,	// 32F
	EnergyVector	= 8,	// 32FC4
	Flow			= 9,	// 16SC2
	Flow2			= 10,	// 16SC2
	Energy			= 10,	// 32F
	Mask			= 11,	// 32U
	Density			= 12,	// 32F
	Support1		= 13,	// 8UC4 (currently)
	Support2		= 14,	// 8UC4 (currently)
	Segmentation	= 15,	// 32S?
	Normals2		= 16,	// 16FC4
	UNUSED1			= 17,	
	Disparity		= 18,
	Smoothing		= 19,	// 32F
	UNUSED2			= 20,
	Overlay			= 21,   // 8UC4
	GroundTruth		= 22,	// 32F

	/* Audio Channels */
	AudioMono		= 32,	// Deprecated, will always be stereo
	AudioStereo		= 33,
	Audio			= 33,

	/* Special data channels */
	Configuration	= 64,	// JSON Data
	Settings1		= 65,
	Calibration		= 65,	// Camera Parameters Object
	Pose			= 66,	// Eigen::Matrix4d, camera transform
	Settings2		= 67,
	Calibration2	= 67,	// Right camera parameters
	Index           = 68,
	Control			= 69,	// For stream and encoder control
	Settings3		= 70,
	MetaData		= 71,	// Map of string pairs (key, value)
	Capabilities	= 72,	// Unordered set of int capabilities
	CalibrationData = 73,	// Just for stereo intrinsics/extrinsics etc
	Thumbnail		= 74,	// Small JPG thumbnail, sometimes updated

	/* Custom / user data channels */
	Data			= 2048,	// Do not use
	EndFrame		= 2048, // Signify the last packet
	Faces			= 2049, // Data about detected faces
	Transforms		= 2050,	// Transformation matrices for framesets
	Shapes3D		= 2051,	// Labeled 3D shapes
	Messages		= 2052,	// Vector of Strings
	Touch			= 2053, // List of touch data type (each touch point)
	Pipelines		= 2054,	// List of pipline URIs that have been applied
};

inline bool isVideo(Channel c) { return (int)c < 32; };
inline bool isAudio(Channel c) { return (int)c >= 32 && (int)c < 64; };
inline bool isData(Channel c) { return (int)c >= 64; };

/** Obtain a string name for channel. */
std::string name(Channel c);

/** Obtain OpenCV type for channel. */
int type(Channel c);

/** @deprecated */
template <int BASE=0>
class Channels {  // TODO: Add [[deprecated]]
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
	inline Channels &operator+=(Channels<BASE> cs) { mask |= cs.mask; return *this; }
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

/** @deprecated */
inline bool isFloatChannel(ftl::codecs::Channel chan) {
	switch (chan) {
	case Channel::GroundTruth:
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

#endif  // _FTL_RGBD_CHANNELS_HPP_
