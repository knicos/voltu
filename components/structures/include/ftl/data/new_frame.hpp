#ifndef _FTL_DATA_NEWFRAME_HPP_
#define _FTL_DATA_NEWFRAME_HPP_

#include <map>
#include <unordered_set>
#include <any>
#include <optional>
#include <list>
#include <unordered_map>
#include <ftl/codecs/channels.hpp>
#include <ftl/exception.hpp>

namespace ftl {
namespace data {

class Session;

/** Kind of channel in terms of data persistence */
enum class ChannelMode {
	PERSISTENT,		// Most recent value, even from previous fram
	IMMEDIATE,		// Only most recent value since last frame
	SEQUENCE		// All changes since last frame
};

struct ChannelConfig {
	std::string name;
	ChannelMode mode;
	size_t type_id;
};

template <typename T>
ChannelConfig make_channel(const std::string &name, ChannelMode mode) {
	// TODO: Generate packer + unpacker?
	return {name, mode, typeid(T).hash_code()};
}

//template <>
//ChannelConfig make_channel<void>(const std::string &name, ChannelMode mode) {
//	return {name, mode, 0};
//}

class Frame {
	public:
	uint32_t id=0;
	int64_t timestamp=0;

	public:
	Frame() : parent_(nullptr) {};
	explicit Frame(Session *parent) : parent_(parent) {};
	~Frame() { flush(); };

	Frame(Frame &&f) {
		f.swapTo(*this);
		f.reset();
	}

	Frame &operator=(Frame &&f) {
		f.swapTo(*this);
		f.reset();
		return *this;
	}

	// Prevent frame copy, instead use a move.
	Frame(const Frame &)=delete;
	Frame &operator=(const Frame &)=delete;

	inline bool has(ftl::codecs::Channel c);

	inline bool changed(ftl::codecs::Channel c);

	inline const std::unordered_set<ftl::codecs::Channel> &changed() const { return changed_; }

	template <typename T>
	bool isType(ftl::codecs::Channel c);

	template <typename T>
	const T &get(ftl::codecs::Channel c) const;

	template <typename T>
	const T *getPtr(ftl::codecs::Channel c) const noexcept;

	template <typename T>
	T *getMutable(ftl::codecs::Channel c);

	inline void touch(ftl::codecs::Channel c) {
		changed_.emplace(c);
	}

	inline void untouch(ftl::codecs::Channel c) {
		changed_.erase(c);
	}

	template <typename T>
	T &create(ftl::codecs::Channel c, const T &value);

	template <typename T>
	T &create(ftl::codecs::Channel c);

	template <typename T, typename ...ARGS>
	T &emplace(ftl::codecs::Channel, ARGS...);

	template <typename T>
	void push(ftl::codecs::Channel c, const T &v);

	template <typename T>
	void set(ftl::codecs::Channel c, const T &v);

	inline void on(ftl::codecs::Channel c, const std::function<bool(Frame&,ftl::codecs::Channel)> &cb);

	inline void on(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb);

	void merge(Frame &);

	void swapTo(Frame &);

	void swapChanged(Frame &);

	void swapChannel(ftl::codecs::Channel, Frame &);

	inline void reset() { changed_.clear(); }

	void clear();

	/** Send changes back through origin stream. */
	bool flush();

	inline auto begin() const { return data_.begin(); }
	inline auto end() const { return data_.end(); }

	// onBeginFlush
	// onEndFlush
	// onError

	static void registerChannel(ftl::codecs::Channel, const ChannelConfig &config);
	static void clearRegistry();

	static bool isPersistent(ftl::codecs::Channel);
	static size_t getChannelType(ftl::codecs::Channel);
	static std::string getChannelName(ftl::codecs::Channel);
	static ftl::codecs::Channel getChannelByName(const std::string &name);

	private:
	std::map<ftl::codecs::Channel, std::any> data_;
	std::unordered_set<ftl::codecs::Channel> changed_;
	std::unordered_map<ftl::codecs::Channel, std::list<std::function<bool(Frame&,ftl::codecs::Channel)>>> triggers_;
	std::list<std::function<bool(Frame&,ftl::codecs::Channel)>> any_triggers_;
	Session *parent_;
};

class Session : public Frame {};

}
}

// ==== Implementations ========================================================

bool ftl::data::Frame::has(ftl::codecs::Channel c) {
	return data_.find(c) != data_.end() || (parent_ && parent_->has(c));
}

bool ftl::data::Frame::changed(ftl::codecs::Channel c) {
	return changed_.find(c) != changed_.end();
}

void ftl::data::Frame::on(ftl::codecs::Channel c, const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
	if (parent_) parent_->on(c, cb);
	else triggers_[c].push_back(cb);  // TODO: Find better way to enable removal
}

void ftl::data::Frame::on(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
	any_triggers_.push_back(cb);
}

template <typename T>
bool ftl::data::Frame::isType(ftl::codecs::Channel c) {
	auto i = data_.find(c);
	if (i != data_.end()) {
		return typeid(T) == i->second.type();
	} else return false;
}

template <typename T>
const T *ftl::data::Frame::getPtr(ftl::codecs::Channel c) const noexcept {
	auto i = data_.find(c);
	if (i != data_.end()) {
		return std::any_cast<T>(&i->second);
	} else if (parent_) {
		return parent_->getPtr<T>(c);
	} else return nullptr;
}

template <typename T>
const T &ftl::data::Frame::get(ftl::codecs::Channel c) const {
	auto i = data_.find(c);
	if (i != data_.end()) {
		auto *p = std::any_cast<T>(&i->second);
		if (!p) throw FTL_Error("'get' wrong type for channel (" << static_cast<unsigned int>(c) << ")");
		return *p;
	} else if (parent_) {
		return parent_->get<T>(c);
	} else throw FTL_Error("'get' on missing channel (" << static_cast<unsigned int>(c) << ")");
}

template <typename T>
T &ftl::data::Frame::create(ftl::codecs::Channel c, const T &value) {
	touch(c);
	size_t t = getChannelType(c);
	if (t > 0 && t != typeid(T).hash_code()) throw FTL_Error("Incorrect type for channel " << static_cast<unsigned int>(c));
	auto &d = data_[c];
	d = value;
	return *std::any_cast<T>(&d);
}

template <typename T>
T &ftl::data::Frame::create(ftl::codecs::Channel c) {
	touch(c);
	size_t t = getChannelType(c);
	if (t > 0 && t != typeid(T).hash_code()) throw FTL_Error("Incorrect type for channel " << static_cast<unsigned int>(c));
	if (!isType<T>(c)) return data_[c].emplace<T>();
	else return *std::any_cast<T>(&data_[c]);
}

template <typename T, typename ...ARGS>
T &ftl::data::Frame::emplace(ftl::codecs::Channel c, ARGS... args) {
	touch(c);
	return data_[c].emplace<T>(std::forward<ARGS...>(args...));
}

template <typename T>
void ftl::data::Frame::push(ftl::codecs::Channel c, const T &v) {
	auto i = data_.find(c);
	if (i != data_.end()) {
		auto *p = std::any_cast<std::vector<T>>(&i->second);
		p->push_back(v);
	} else {
		throw FTL_Error("Push on missing channel (" << static_cast<unsigned int>(c) << ")");
	}
	touch(c);
}

template <typename T>
void ftl::data::Frame::set(ftl::codecs::Channel c, const T &v) {
	auto i = data_.find(c);
	if (i != data_.end()) {
		i->second = v;
	} else if (parent_ && parent_->isType<T>(c)) {
		create<T>(c, v);
	} else {
		throw FTL_Error("Set on missing channel (" << static_cast<unsigned int>(c) << ")");
	}
	touch(c);
}

#endif