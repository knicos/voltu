/**
 * @file transactional.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_TRANSACTIONAL_HPP_
#define _FTL_TRANSACTIONAL_HPP_

#include <ftl/threads.hpp>

namespace ftl {

/**
 * Use RAII style transactional objects with shared locking. This wraps an
 * object with a lock and provides a release notification mechanism to allow
 * completion code.
 * 
 * Used by ftl::stream::Receiver and Builder.
 */
template <typename T>
class Transactional {
	static_assert(std::is_pointer<T>::value, "Transactional type must be a pointer");

	public:
	Transactional() : ref_(nullptr), mtx_(nullptr) {}
	Transactional(T obj, SHARED_MUTEX *mtx) : ref_(obj), mtx_(mtx), lock_(*mtx_) {}
	Transactional(T obj, SHARED_MUTEX *mtx, const std::function<void(T)> &complete) : ref_(obj), mtx_(mtx), lock_(*mtx_), completed_(complete) {}
	Transactional(const Transactional &)=delete;
	~Transactional() {
		if (lock_) lock_.unlock();
		if (completed_) completed_(ref_);
	}

	Transactional(Transactional &&t) : ref_(t.ref_), mtx_(t.mtx_), lock_(*mtx_), completed_(t.completed_) {
		t.completed_ = nullptr;
	}

	Transactional &operator=(const Transactional &)=delete;

	bool isValid() const { return ref_ != nullptr; }
	operator bool() const { return ref_ != nullptr; }

	T operator->() { if (!ref_) throw FTL_Error("Use of invalid frameset"); return ref_; }
	const T operator->() const { if (!ref_) throw FTL_Error("Use of invalid frameset"); return ref_; }

	T operator*() { if (!ref_) throw FTL_Error("Use of invalid frameset"); return ref_; }
	const T operator*() const { if (!ref_) throw FTL_Error("Use of invalid frameset"); return ref_; }

	private:
	T ref_;
	SHARED_MUTEX *mtx_;
	SHARED_LOCK_TYPE(SHARED_MUTEX) lock_;
	std::function<void(T)> completed_;
};

}

#endif
