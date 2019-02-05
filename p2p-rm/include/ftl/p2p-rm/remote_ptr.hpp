#ifndef _FTL_P2P_RA_REMOTE_PTR_HPP_
#define _FTL_P2P_RA_REMOTE_PTR_HPP_

#include "ftl/p2p-rm/blob.hpp"

namespace ftl {
	template <typename T> struct write_ref;
	template <typename T> struct read_ref;
	template <typename T> struct write_ptr;
	template <typename T> struct read_ptr;
	
	template <typename T>
	struct remote_ptr {
		rm::Blob *blob;
		size_t offset;
		
		bool is_null() const { return blob == NULL; }
		bool is_local() const { return blob->data_ != NULL; }
		bool is_valid() const {
			return !is_null() && offset+sizeof(T) <= blob->size_;
		}
		
		T *get() { return blob->data_; }
		size_t size() const { return blob->size_; }
		
		write_ref<T> operator*();
		write_ref<T> operator[](ptrdiff_t idx);
		write_ref<T> writable() { return ftl::write_ref<T>(*this); }
		
		read_ref<T> operator*() const;
		read_ref<T> operator[](ptrdiff_t idx) const;
		read_ref<T> readable() { return ftl::read_ref<T>(*this); }
		
		remote_ptr<T> operator+(std::ptrdiff_t diff) const {
			size_t new_offset = offset + sizeof(T)*diff;
			return remote_ptr<T>{blob, new_offset};
		}
		
		/** Allow pointer casting if a local blob */
		template <typename U>
		operator U*() {
			if (is_local()) return (U*)blob->data_;
			return NULL;
		}
	};
	
	template <typename T>
	struct read_ref {
		remote_ptr<T> ptr_;
		
		// Constructor
		read_ref(remote_ptr<T> ptr) : ptr_(ptr), rlock_(ptr.blob->mutex_) {}
		
		bool is_valid() const { return !ptr_.is_null(); }
		remote_ptr<T> pointer() const { return ptr_; }
		void reset() { rlock_.unlock(); }
		void finish() { reset(); }
		
		operator T() const {
			//return static_cast<T>(ptr_.blob->data_[ptr_.offset]);
			T t;
			ptr_.blob->read(ptr_.offset, (char*)&t, sizeof(T));
			return t;
		}
		
		read_ref &operator=(const T &value) {
			// silent fail!
			return *this;
		}
		
		std::shared_lock<std::shared_mutex> rlock_;
	};
	
	template <typename T>
	struct write_ref {
		remote_ptr<T> ptr_;
		
		// Constructor
		write_ref(remote_ptr<T> ptr) : ptr_(ptr), wlock_(ptr.blob->mutex_) {}
		~write_ref() { ptr_.blob->finished(); }
		
		bool is_valid() const { return !ptr_.is_null(); }
		remote_ptr<T> pointer() const { return ptr_; }
		void reset() { ptr_.blob->finished(); wlock_.unlock(); }
		void finish() { reset(); }
		
		/** Cast to type reads the value */
		operator T() const {
			//return static_cast<T>(ptr_.blob->data_[ptr_.offset]);
			T t;
			ptr_.blob->read(ptr_.offset, (char*)&t, sizeof(T));
			return t;
		}
		
		write_ref &operator=(const T &value) {
			ptr_.blob->write(ptr_.offset, (char*)(&value), sizeof(T));
			return *this;
		}
		
		std::unique_lock<std::shared_mutex> wlock_;
	};
}

template <typename T>
ftl::read_ref<T> ftl::remote_ptr<T>::operator*() const {
	return ftl::read_ref<T>(*this);
}

template <typename T>
ftl::read_ref<T> ftl::remote_ptr<T>::operator[](ptrdiff_t idx) const {
	return ftl::read_ref<T>(*this + idx);
}

template <typename T>
ftl::write_ref<T> ftl::remote_ptr<T>::operator*() {
	return ftl::write_ref<T>(*this);
}

template <typename T>
ftl::write_ref<T> ftl::remote_ptr<T>::operator[](ptrdiff_t idx) {
	return ftl::write_ref<T>(*this + idx);
}

#endif // _FTL_P2P_RA_REMOTE_PTR_HPP_

