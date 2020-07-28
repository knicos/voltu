#ifndef _FTL_DATA_CHANNELS_HPP_
#define _FTL_DATA_CHANNELS_HPP_

#include <string>
#include <ftl/codecs/channels.hpp>
#include <ftl/exception.hpp>
#include <ftl/utility/vectorbuffer.hpp>

namespace ftl {
namespace data {

class Frame;

/** Kind of channel in terms of data persistence */
enum class StorageMode {
	PERSISTENT,		// Most recent value, even from previous fram
	TRANSIENT,		// Only most recent value since last frame
	AGGREGATE		// All changes since last frame
};

/** If a channel has changed, what is the current status of that change. */
enum class ChangeType {
	UNCHANGED,
	PRIMARY,		// Explicit local primary modification occurred
	RESPONSE,		// Explicit local response change
	FOREIGN,		// Received externally, to be forwarded
	COMPLETED		// Received externally, not to be forwarded
};

/** Current status of the data contained within a channel. */
enum class ChannelStatus {
	INVALID,		// Any data is stale and should not be referenced
	VALID,			// Contains currently valid data
	FLUSHED,		// Has already been transmitted, now read-only
	DISPATCHED,		// Externally received, can't be flushed but can be modified locally
	ENCODED			// Still in an encoded form
};

/* Internal structure for channel configurations. */
struct ChannelConfig {
	std::string name;
	StorageMode mode;
	size_t type_id;
};

/**
 * Add a channel configuration to the registry. By default channels are not
 * in the registry and this means they have no name or specified type. Non
 * registered channels can still be used but no runtime checks are performed.
 */
void registerChannel(ftl::codecs::Channel, const ChannelConfig &config);

/** Used by unit tests. */
void clearRegistry();

/**
 * Check if channel is marked as persistent storage in the registry.
 */
bool isPersistent(ftl::codecs::Channel);

bool isAggregate(ftl::codecs::Channel);

/**
 * Get channel type hash_code as from `std::type_info::hash_code()`. This
 * returns 0 if not registered or registered as allowing any time, 0 means
 * accept any type.
 */
size_t getChannelType(ftl::codecs::Channel);

template <typename T>
void verifyChannelType(ftl::codecs::Channel c) {
	size_t t = getChannelType(c);
	if (t > 0 && t != typeid(T).hash_code()) throw FTL_Error("Incorrect type for channel " << static_cast<unsigned int>(c));
}

/**
 * Get the registered string name for channel, or an empty string if no name.
 */
std::string getChannelName(ftl::codecs::Channel);

/** Unsupported */
ftl::codecs::Channel getChannelByName(const std::string &name);

/**
 * Attempts to get a msgpack encoder for this channel. Such encoders are
 * registered by typeid basis when creating channels.
 */
std::function<bool(const ftl::data::Frame &, ftl::codecs::Channel, std::vector<uint8_t> &)> getTypeEncoder(size_t type);

void setTypeEncoder(size_t type, const std::function<bool(const ftl::data::Frame &, ftl::codecs::Channel, std::vector<uint8_t> &)> &e);

/**
 * Helper to register a channel using a template specified type.
 */
template <typename T>
bool make_channel(ftl::codecs::Channel c, const std::string &name, StorageMode mode) {
	// TODO: Generate packer + unpacker?
	registerChannel(c, {name, mode, typeid(T).hash_code()});
	return true;
}

template <>
inline bool make_channel<void>(ftl::codecs::Channel c, const std::string &name, StorageMode mode) {
	registerChannel(c, {name, mode, 0});
	return true;
}

}
}

#endif
