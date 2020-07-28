#ifndef _FTL_HANDLE_HPP_
#define _FTL_HANDLE_HPP_

#include <ftl/threads.hpp>
#include <ftl/exception.hpp>
#include <functional>
#include <unordered_map>

namespace ftl {

struct Handle;
struct BaseHandler {
	virtual void remove(const Handle &)=0;

	inline Handle make_handle(BaseHandler*, int);

	protected:
	std::mutex mutex_;
	int id_=0;
};

/**
 * A `Handle` is used to manage registered callbacks, allowing them to be
 * removed safely whenever the `Handle` instance is destroyed.
 */
struct [[nodiscard]] Handle {
	friend struct BaseHandler;

	/**
	 * Cancel the callback and invalidate the handle.
	 */
	inline void cancel() { if (handler_) handler_->remove(*this); handler_ = nullptr; }

	inline int id() const { return id_; }

	Handle() : handler_(nullptr), id_(0) {}

	Handle(const Handle &)=delete;
	Handle &operator=(const Handle &)=delete;

	inline Handle(Handle &&h) : handler_(nullptr) {
		if (handler_) handler_->remove(*this);
		handler_ = h.handler_;
		h.handler_ = nullptr;
		id_ = h.id_;
	}

	inline Handle &operator=(Handle &&h) {
		if (handler_) handler_->remove(*this);
		handler_ = h.handler_;
		h.handler_ = nullptr;
		id_ = h.id_;
		return *this;
	}

	inline ~Handle() {
		if (handler_) {
			handler_->remove(*this);
		}
	}

	private:
	BaseHandler *handler_;
	int id_;

	Handle(BaseHandler *h, int id) : handler_(h), id_(id) {}
};

/**
 * This class is used to manage callbacks. The template parameters are the
 * arguments to be passed to the callback when triggered. This class is already
 * thread-safe.
 *
 * POSSIBLE BUG:	On destruction any remaining handles will be left with
 * 					dangling pointer to Handler.
 */
template <typename ...ARGS>
struct Handler : BaseHandler {

	/**
	 * Add a new callback function. It returns a `Handle` object that must
	 * remain in scope, the destructor of the `Handle` will remove the callback.
	 */
	Handle on(const std::function<bool(ARGS...)> &f) {
		std::unique_lock<std::mutex> lk(mutex_);
		int id = id_++;
		callbacks_[id] = f;
		return make_handle(this, id);
	}

	/**
	 * Safely trigger all callbacks. Note that `Handler` is locked when
	 * triggering so callbacks cannot make modifications to it or they will
	 * lock up. To remove a callback, return false from the callback, else
	 * return true.
	 */
	void trigger(ARGS ...args) {
		std::unique_lock<std::mutex> lk(mutex_);
		//try {
			for (auto i=callbacks_.begin(); i!=callbacks_.end(); ) {
				bool keep = i->second(std::forward<ARGS>(args)...);
				if (!keep) i = callbacks_.erase(i);
				else ++i;
			}
		//} catch (const std::exception &e) {
		//	LOG(ERROR) << "Exception in callback: " << e.what();
		//}
	}

	/**
	 * Remove a callback using its `Handle`. This is equivalent to allowing the
	 * `Handle` to be destroyed or cancelled.
	 */
	void remove(const Handle &h) override {
		std::unique_lock<std::mutex> lk(mutex_);
		callbacks_.erase(h.id());
	}

	private:
	std::unordered_map<int, std::function<bool(ARGS...)>> callbacks_;
};

/**
 * This class is used to manage callbacks. The template parameters are the
 * arguments to be passed to the callback when triggered. This class is already
 * thread-safe. Note that this version only allows a single callback at a time
 * and throws an exception if multiple are added without resetting.
 */
template <typename ...ARGS>
struct SingletonHandler : BaseHandler {
	/**
	 * Add a new callback function. It returns a `Handle` object that must
	 * remain in scope, the destructor of the `Handle` will remove the callback.
	 */
	[[nodiscard]] Handle on(const std::function<bool(ARGS...)> &f) {
		std::unique_lock<std::mutex> lk(mutex_);
		if (callback_) throw FTL_Error("Callback already bound");
		callback_ = f;
		return make_handle(this, id_++);
	}

	/**
	 * Safely trigger all callbacks. Note that `Handler` is locked when
	 * triggering so callbacks cannot make modifications to it or they will
	 * lock up. To remove a callback, return false from the callback, else
	 * return true.
	 */
	bool trigger(ARGS ...args) {
		std::unique_lock<std::mutex> lk(mutex_);
		if (callback_) {
			bool keep = callback_(std::forward<ARGS>(args)...);
			if (!keep) callback_ = nullptr;
			return keep;
		} else {
			return false;
		}
		//} catch (const std::exception &e) {
		//	LOG(ERROR) << "Exception in callback: " << e.what();
		//}
	}

	/**
	 * Remove a callback using its `Handle`. This is equivalent to allowing the
	 * `Handle` to be destroyed or cancelled. If the handle does not match the
	 * currently bound callback then the callback is not removed.
	 */
	void remove(const Handle &h) override {
		std::unique_lock<std::mutex> lk(mutex_);
		if (h.id() == id_-1) callback_ = nullptr;
	}

	void reset() { callback_ = nullptr; }

	operator bool() const { return (bool)callback_; }

	private:
	std::function<bool(ARGS...)> callback_;
};

}

ftl::Handle ftl::BaseHandler::make_handle(BaseHandler *h, int id) {
	return ftl::Handle(h, id);
}

#endif
