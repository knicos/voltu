#ifndef _FTL_DATA_NEWFRAME_HPP_
#define _FTL_DATA_NEWFRAME_HPP_

#include <map>
#include <unordered_set>
#include <any>
#include <optional>
#include <list>
#include <unordered_map>
#include <ftl/codecs/channels.hpp>
#include <ftl/data/channels.hpp>
#include <ftl/exception.hpp>
#include <ftl/handle.hpp>

template<typename T> struct is_list : public std::false_type {};

template<typename T>
struct is_list<std::list<T>> : public std::true_type {};

namespace ftl {
namespace streams { class Feed; }
namespace data {

class Session;
class Pool;

enum FrameStatus {
	CREATED,   // Initial state, before store
	STORED,    // Changed to this after call to `store`
	FLUSHED,   // Changed to this after call to `flush`
	RELEASED   // Destroyed or moved
};

/**
 * Helper class to enable aggregation of aggregate channels.
 */
template <typename T>
struct Aggregator {
	T &list;
	bool aggregate=true;

	Aggregator &operator=(const T &l) {
		if (aggregate) list.insert(list.end(), l.begin(), l.end());
		else list = l;
		return *this;
	}

	Aggregator &operator=(T &&l) {
		if (aggregate) list.splice(list.end(), l, l.begin(), l.end());
		else list = std::move(l);
		return *this;
	}

	operator T() { return list; }
	operator T() const { return list; }
};

class Frame {
	friend class Session;
	friend class ftl::data::Pool;
	friend class ftl::streams::Feed;

	private:
	// Only Feed class should construct
	Frame(Pool *ppool, Session *parent, uint32_t pid, int64_t ts) : timestamp_(ts), id(pid), pool_(ppool), parent_(parent), status_(FrameStatus::CREATED) {};
	int64_t timestamp_=0;

	public:
	const uint32_t id=0;

	inline int64_t timestamp() const { return timestamp_; }

	public:
	Frame()=delete;
	
	~Frame();

	Frame(Frame &&f) {
		f.moveTo(*this);
	}

	Frame &operator=(Frame &&f) {
		f.moveTo(*this);
		return *this;
	}

	// Prevent frame copy, instead use a move.
	Frame(const Frame &)=delete;
	Frame &operator=(const Frame &)=delete;

	inline FrameStatus status() const { return status_; }

	inline size_t size() const { return data_.size(); }

	bool has(ftl::codecs::Channel c) const;

	inline bool hasOwn(ftl::codecs::Channel c) const;

	inline bool changed(ftl::codecs::Channel c) const;

	inline bool readonly(ftl::codecs::Channel c) const;

	inline bool flushed(ftl::codecs::Channel c) const;

	inline ftl::data::ChangeType getChangeType(ftl::codecs::Channel c) const;

	inline const std::unordered_map<ftl::codecs::Channel, ChangeType> &changed() const { return changed_; }

	template <typename T>
	bool isType(ftl::codecs::Channel c) const;

	template <typename T>
	const T &get(ftl::codecs::Channel c) const;

	const std::any &getAny(ftl::codecs::Channel c) const;

	std::any &getAnyMutable(ftl::codecs::Channel c);

	template <typename T>
	const T *getPtr(ftl::codecs::Channel c) const noexcept;

	template <typename T>
	T *getMutable(ftl::codecs::Channel c);

	inline void touch(ftl::codecs::Channel c) {
		changed_[c] = ChangeType::LOCAL;
	}

	inline void touch(ftl::codecs::Channel c, ChangeType type) {
		changed_[c] = type;
	}

	inline void untouch(ftl::codecs::Channel c) {
		changed_.erase(c);
	}

	template <typename T, std::enable_if_t<!is_list<T>::value,int> = 0>
	T &create(ftl::codecs::Channel c);

	template <typename T, std::enable_if_t<is_list<T>::value,int> = 0>
	ftl::data::Aggregator<T> create(ftl::codecs::Channel c);

	/**
	 * Creates a channel data entry with a forced change status. This also
	 * changes the channel status to `DISPATCHED`. If the storage mode is
	 * `persistent` this adds to session store instead of local frame store,
	 * although the change status is added to the local frame.
	 * 
	 * To be used by receive, no one else.
	 */
	template <typename T, std::enable_if_t<!is_list<T>::value,int> = 0>
	T &createChange(ftl::codecs::Channel c, ftl::data::ChangeType t);

	template <typename T, std::enable_if_t<is_list<T>::value,int> = 0>
	ftl::data::Aggregator<T> createChange(ftl::codecs::Channel c, ftl::data::ChangeType t);

	template <typename T>
	T &createChange(ftl::codecs::Channel c, ftl::data::ChangeType t, std::vector<uint8_t> &data);

	const std::vector<uint8_t> &getEncoded(ftl::codecs::Channel c) const;

	template <typename T, typename ...ARGS>
	T &emplace(ftl::codecs::Channel, ARGS...);

	template <typename T, std::enable_if_t<!is_list<T>::value,int> = 0>
	T &set(ftl::codecs::Channel c);

	template <typename T, std::enable_if_t<is_list<T>::value,int> = 0>
	ftl::data::Aggregator<T> set(ftl::codecs::Channel c);

	/**
	 * Will remove a channel by changing its status and will not remove data.
	 */
	void remove(ftl::codecs::Channel);

	/**
	 * Will remove a channel and destroy all data associated with it.
	 */
	void hardRemove(ftl::codecs::Channel);

	inline ftl::Handle onChange(ftl::codecs::Channel c, const std::function<bool(Frame&,ftl::codecs::Channel)> &cb);

	inline ftl::Handle onChange(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb);

	inline ftl::Handle onFlush(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb);

	/**
	 * Merge the given frame parameter into this frame. It is a move operation
	 * on a per channel basis.
	 */
	void merge(Frame &);

	void moveTo(Frame &);

	void swapChanged(Frame &);

	void swapChannel(ftl::codecs::Channel, Frame &);

	/**
	 * Discard all change status without removing the data.
	 */
	inline void resetChanges() { changed_.clear(); }

	/**
	 * Clears all state to an empty condition without releasing memory.
	 */
	void reset();

	/**
	 * Deletes all memory and resets to starting condition. This should not
	 * be used, instead use `release()` which will save the memory into a pool
	 * rather than deleting it completely.
	 */
	void hardReset();

	/**
	 * Free memory into the memory pool. This also implicitly resets.
	 */
	void release();

	/** Send changes back through origin stream. */
	bool flush();

	bool flush(ftl::codecs::Channel c);

	/** Copy persistent changes to session. To be called before dispatch. */
	void store();

	inline auto begin() const { return data_.begin(); }
	inline auto end() const { return data_.end(); }

	// onBeginFlush
	// onEndFlush
	// onError

	inline MUTEX &mutex();

	protected:
	std::any &createAnyChange(ftl::codecs::Channel c, ftl::data::ChangeType t);

	std::any &createAnyChange(ftl::codecs::Channel c, ftl::data::ChangeType t, std::vector<uint8_t> &data);

	std::any &createAny(ftl::codecs::Channel c);

	private:
	struct ChannelData {
		ChannelStatus status=ChannelStatus::INVALID;
		std::any data;
		std::vector<uint8_t> encoded={};
	};

	ChannelData &_getData(ftl::codecs::Channel);
	const ChannelData &_getData(ftl::codecs::Channel) const;

	std::map<ftl::codecs::Channel, ChannelData> data_;
	std::unordered_map<ftl::codecs::Channel, ChangeType> changed_;
	Pool *pool_;
	Session *parent_;
	FrameStatus status_;

	inline void restart(int64_t ts) {
		timestamp_ = ts;
		status_ = FrameStatus::CREATED;
	}
};

class Session : public Frame {
	friend class Frame;

	public:
	Session() : Frame(nullptr, nullptr,0,0) {
		status_ = FrameStatus::STORED;
	}

	~Session() {
		// Sessions don't get memory pooled.
		status_ = FrameStatus::RELEASED;
	}

	ftl::Handle onChange(uint32_t id, ftl::codecs::Channel c, const std::function<bool(Frame&,ftl::codecs::Channel)> &cb);

	ftl::Handle onChange(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb);

	ftl::Handle onFlush(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb);

	void notifyChanges(Frame &f);

	void flush(Frame &f);

	void flush(Frame &f, ftl::codecs::Channel c);

	inline MUTEX &mutex() { return mutex_; }
	
	private:
	std::unordered_map<uint64_t, ftl::Handler<Frame&,ftl::codecs::Channel>> change_channel_;
	ftl::Handler<Frame&,ftl::codecs::Channel> change_;
	ftl::Handler<Frame&,ftl::codecs::Channel> flush_;

	MUTEX mutex_;
};

}
}

// ==== Implementations ========================================================

MUTEX &ftl::data::Frame::mutex() { return parent_->mutex(); }

bool ftl::data::Frame::hasOwn(ftl::codecs::Channel c) const {
	const auto &i = data_.find(c);
	return (i != data_.end() && i->second.status != ftl::data::ChannelStatus::INVALID);
}

bool ftl::data::Frame::changed(ftl::codecs::Channel c) const {
	return changed_.find(c) != changed_.end();
}

ftl::data::ChangeType ftl::data::Frame::getChangeType(ftl::codecs::Channel c) const {
	const auto &i = changed_.find(c);
	return (i == changed_.end()) ? ftl::data::ChangeType::UNCHANGED : i->second;
}

bool ftl::data::Frame::flushed(ftl::codecs::Channel c) const {
	const auto &d = _getData(c);
	return d.status == ChannelStatus::FLUSHED;
}

bool ftl::data::Frame::readonly(ftl::codecs::Channel c) const {
	return flushed(c);
}

ftl::Handle ftl::data::Frame::onChange(ftl::codecs::Channel c, const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
	return parent_->onChange(id, c, cb);
}

ftl::Handle ftl::data::Frame::onChange(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
	return parent_->onChange(cb);
}

ftl::Handle ftl::data::Frame::onFlush(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
	return parent_->onFlush(cb);
}

template <typename T>
bool ftl::data::Frame::isType(ftl::codecs::Channel c) const {
	const auto &i = data_.find(c);
	if (i != data_.end() && i->second.status != ftl::data::ChannelStatus::INVALID) {
		return typeid(T) == i->second.data.type();
	} else {
		return (parent_ && parent_->isType<T>(c)); 
	}
}

template <typename T>
const T *ftl::data::Frame::getPtr(ftl::codecs::Channel c) const noexcept {
	const auto &d = _getData(c);
	if (d.status != ftl::data::ChannelStatus::INVALID) {
		return std::any_cast<T>(&d.data);
	} else return nullptr;
}

template <typename T>
const T &ftl::data::Frame::get(ftl::codecs::Channel c) const {
	const auto &d = _getData(c);
	if (d.status != ftl::data::ChannelStatus::INVALID) {
		auto *p = std::any_cast<T>(&d.data);
		if (!p) throw FTL_Error("'get' wrong type for channel (" << static_cast<unsigned int>(c) << ")");
		return *p;
	} else throw FTL_Error("Missing channel (" << static_cast<unsigned int>(c) << ")");
}

// Non-list version
template <typename T, std::enable_if_t<!is_list<T>::value,int> = 0>
T &ftl::data::Frame::create(ftl::codecs::Channel c) {
	if (isAggregate(c)) throw FTL_Error("Aggregate channels must be of list type");

	ftl::data::verifyChannelType<T>(c);

	std::any &a = createAny(c);
	if (!isType<T>(c)) return a.emplace<T>();
	else return *std::any_cast<T>(&a);
}

// List version
template <typename T, std::enable_if_t<is_list<T>::value,int> = 0>
ftl::data::Aggregator<T> ftl::data::Frame::create(ftl::codecs::Channel c) {
	ftl::data::verifyChannelType<T>(c);

	std::any &a = createAny(c);
	if (!isType<T>(c)) a.emplace<T>();
	return ftl::data::Aggregator<T>{*std::any_cast<T>(&a), isAggregate(c)};
}

template <typename T>
T &ftl::data::Frame::createChange(ftl::codecs::Channel c, ftl::data::ChangeType type, std::vector<uint8_t> &data) {
	if (!bool(is_list<T>{}) && isAggregate(c)) throw FTL_Error("Aggregate channels must be of list type");

	ftl::data::verifyChannelType<T>(c);

	std::any &a = createAnyChange(c, type, data);
	if (!isType<T>(c)) return a.emplace<T>();
	else return *std::any_cast<T>(&a);
}

// Non-list version
template <typename T, std::enable_if_t<!is_list<T>::value,int> = 0>
T &ftl::data::Frame::createChange(ftl::codecs::Channel c, ftl::data::ChangeType type) {
	if (isAggregate(c)) throw FTL_Error("Aggregate channels must be of list type");

	ftl::data::verifyChannelType<T>(c);

	std::any &a = createAnyChange(c, type);
	if (!isType<T>(c)) return a.emplace<T>();
	else return *std::any_cast<T>(&a);
}

// List version
template <typename T, std::enable_if_t<is_list<T>::value,int> = 0>
ftl::data::Aggregator<T> ftl::data::Frame::createChange(ftl::codecs::Channel c, ftl::data::ChangeType type) {
	ftl::data::verifyChannelType<T>(c);

	std::any &a = createAnyChange(c, type);
	if (!isType<T>(c)) a.emplace<T>();
	return ftl::data::Aggregator<T>{*std::any_cast<T>(&a), isAggregate(c)};
}

template <typename T, typename ...ARGS>
T &ftl::data::Frame::emplace(ftl::codecs::Channel c, ARGS... args) {
	touch(c);
	return data_[c].data.emplace<T>(std::forward<ARGS...>(args...));
}

// Non-list version
template <typename T, std::enable_if_t<!is_list<T>::value,int> = 0>
T &ftl::data::Frame::set(ftl::codecs::Channel c) {
	if (status_ != FrameStatus::STORED) throw FTL_Error("Cannot modify before store");

	auto i = data_.find(c);
	if (i != data_.end()) {
		if (i->second.status != ftl::data::ChannelStatus::FLUSHED) {
			i->second.encoded.clear();
			touch(c);
			return *std::any_cast<T>(&i->second.data);
		} else {
			throw FTL_Error("Channel is flushed and read-only: " << static_cast<unsigned int>(c));
		}
	} else if (parent_ && parent_->isType<T>(c)) {
		touch(c);
		return create<T>(c);
	} else {
		throw FTL_Error("Set on missing channel (" << static_cast<unsigned int>(c) << ")");
	}
}

// List version
template <typename T, std::enable_if_t<is_list<T>::value,int> = 0>
ftl::data::Aggregator<T> ftl::data::Frame::set(ftl::codecs::Channel c) {
	if (status_ != FrameStatus::STORED) throw FTL_Error("Cannot modify before store");

	auto i = data_.find(c);
	if (i != data_.end()) {
		if (i->second.status != ftl::data::ChannelStatus::FLUSHED) {
			i->second.encoded.clear();
			touch(c);
			return ftl::data::Aggregator<T>{*std::any_cast<T>(&i->second.data), isAggregate(c)};
		} else {
			throw FTL_Error("Channel is flushed and read-only: " << static_cast<unsigned int>(c));
		}
	} else if (parent_ && parent_->isType<T>(c)) {
		touch(c);
		return create<T>(c);
	} else {
		throw FTL_Error("Set on missing channel (" << static_cast<unsigned int>(c) << ")");
	}
}

#endif