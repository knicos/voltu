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

class Frame {
	public:
	uint32_t id=0;
	int64_t timestamp=0;

	public:
	Frame() : parent_(nullptr) {};
	explicit Frame(Frame *parent) : parent_(parent) {};
	~Frame() { flush(); };

	inline bool has(ftl::codecs::Channel c) {
		return data_.find(c) != data_.end() || (parent_ && parent_->has(c));
	}

	inline bool changed(ftl::codecs::Channel c) {
		return changed_.find(c) != changed_.end();
	}

	inline const std::unordered_set<ftl::codecs::Channel> &changed() const { return changed_; }

	template <typename T>
	bool isType(ftl::codecs::Channel c);

	template <typename T>
	const T &get(ftl::codecs::Channel c) const;

	template <typename T>
	const T *getPtr(ftl::codecs::Channel c) const;

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

	inline void on(ftl::codecs::Channel c, const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
		if (parent_) parent_->on(c, cb);
		else triggers_[c].push_back(cb);  // TODO: Find better way to enable removal
	}

	inline void on(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
		any_triggers_.push_back(cb);
	}

	void merge(Frame &);

	void swap(Frame &);

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

	private:
	std::map<ftl::codecs::Channel, std::any> data_;
	std::unordered_set<ftl::codecs::Channel> changed_;
	std::unordered_map<ftl::codecs::Channel, std::list<std::function<bool(Frame&,ftl::codecs::Channel)>>> triggers_;
	std::list<std::function<bool(Frame&,ftl::codecs::Channel)>> any_triggers_;
	Frame *parent_;
};

}
}

// ==== Implementations ========================================================

template <typename T>
bool ftl::data::Frame::isType(ftl::codecs::Channel c) {
	auto i = data_.find(c);
	if (i != data_.end()) {
		return typeid(T) == i->second.type();
	} else return false;
}

template <typename T>
const T *ftl::data::Frame::getPtr(ftl::codecs::Channel c) const {
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
	auto &d = data_[c];
	d = value;
	return *std::any_cast<T>(&d);
}

template <typename T>
T &ftl::data::Frame::create(ftl::codecs::Channel c) {
	touch(c);
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