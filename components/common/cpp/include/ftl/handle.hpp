#ifndef _FTL_HANDLE_HPP_
#define _FTL_HANDLE_HPP_

#include <ftl/threads.hpp>
#include <functional>
#include <unordered_map>

namespace ftl {

struct Handle;
struct BaseHandler {
	virtual void remove(const Handle &)=0;

	inline Handle make_handle(BaseHandler*, int);

	protected:
	MUTEX mutex_;
	int id_=0;
};

/**
 * A `Handle` is used to manage registered callbacks, allowing them to be
 * removed safely whenever the `Handle` instance is destroyed.
 */
struct Handle {
	friend struct BaseHandler;

	/**
	 * Cancel the timer job. If currently executing it will block and wait for
	 * the job to complete.
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

	inline ~Handle() { if (handler_) handler_->remove(*this); }

	private:
	BaseHandler *handler_;
	int id_;

	Handle(BaseHandler *h, int id) : handler_(h), id_(id) {}
};

template <typename ...ARGS>
struct Handler : BaseHandler {
	Handle on(const std::function<bool(ARGS...)> &f) {
		std::unique_lock<std::mutex> lk(mutex_);
		int id = id_++;
		callbacks_[id] = f;
		return make_handle(this, id);
	}

	void trigger(ARGS ...args) {
		std::unique_lock<std::mutex> lk(mutex_);
		try {
			for (auto &f : callbacks_) {
				f.second(std::forward<ARGS...>(args...));
			}
		} catch (const std::exception &e) {
			LOG(ERROR) << "Exception in callback: " << e.what();
		}
	}

	void remove(const Handle &h) override {
		std::unique_lock<std::mutex> lk(mutex_);
		callbacks_.erase(h.id());
	}

	private:
	std::unordered_map<int, std::function<bool(ARGS...)>> callbacks_;
};

}

ftl::Handle ftl::BaseHandler::make_handle(BaseHandler *h, int id) {
	return ftl::Handle(h, id);
}

#endif