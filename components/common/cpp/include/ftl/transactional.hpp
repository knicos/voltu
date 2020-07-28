#ifndef _FTL_TRANSACTIONAL_HPP_
#define _FTL_TRANSACTIONAL_HPP_

#include <ftl/threads.hpp>

namespace ftl {

/**
 * Use RAII style transactional objects with shared locking. This wraps an
 * object with a lock and provides a release notification mechanism to allow
 * completion code.
 */
template <typename T>
class Transactional {
	static_assert(std::is_pointer<T>::value, "Transactional type must be a pointer");

	public:
	Transactional(T obj, SHARED_MUTEX &mtx) : ref_(obj), mtx_(mtx), lock_(mtx_) {}
	Transactional(T obj, SHARED_MUTEX &mtx, const std::function<void(T)> &complete) : ref_(obj), mtx_(mtx), lock_(mtx_), completed_(complete) {}
	Transactional(const Transactional &)=delete;
	Transactional()=delete;
	~Transactional() {
		lock_.unlock();
		if (completed_) completed_(ref_);
	}

	Transactional(Transactional &&t) : ref_(t.ref_), mtx_(t.mtx_), lock_(mtx_), completed_(t.completed_) {
		t.completed_ = nullptr;
	}

	Transactional &operator=(const Transactional &)=delete;

	T operator->() { return ref_; }
	const T operator->() const { return ref_; }

	T operator*() { return ref_; }
	const T operator*() const { return ref_; }

	private:
	T ref_;
	SHARED_MUTEX &mtx_;
	SHARED_LOCK_TYPE(SHARED_MUTEX) lock_;
	std::function<void(T)> completed_;
};

}

#endif
