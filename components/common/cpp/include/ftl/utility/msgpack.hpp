/**
 * @file msgpack.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

/* Extend msgpack for OpenCV and Eigen types */

#ifndef _FTL_MSGPACK_HPP_
#define _FTL_MSGPACK_HPP_

#ifdef _MSC_VER
#include "msgpack_optional.hpp"
#endif

#include <msgpack.hpp>
#include <opencv2/core/mat.hpp>
#include <Eigen/Eigen>

namespace msgpack {
MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
namespace adaptor {

////////////////////////////////////////////////////////////////////////////////
// cv::Size_<T>

template<typename T>
struct pack<cv::Size_<T>> {
	template <typename Stream>
	packer<Stream>& operator()(msgpack::packer<Stream>& o, cv::Size_<T> const& v) const {

		o.pack_array(2);
		o.pack(v.width);
		o.pack(v.height);

		return o;
	}
};

template<typename T>
struct convert<cv::Size_<T>> {
	msgpack::object const& operator()(msgpack::object const& o, cv::Size_<T>& v) const {
		if (o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }
		if (o.via.array.size != 2) { throw msgpack::type_error(); }

		T width = o.via.array.ptr[0].as<T>();
		T height = o.via.array.ptr[1].as<T>();
		v = cv::Size_<T>(width, height);
		return o;
	}
};

template <typename T>
struct object_with_zone<cv::Size_<T>> {
	void operator()(msgpack::object::with_zone& o, cv::Size_<T> const& v) const {
		o.type = type::ARRAY;
		o.via.array.size = 2;
		o.via.array.ptr = static_cast<msgpack::object*>(
			o.zone.allocate_align(	sizeof(msgpack::object) * o.via.array.size,
									MSGPACK_ZONE_ALIGNOF(msgpack::object)));

		o.via.array.ptr[0] = msgpack::object(v.width, o.zone);
		o.via.array.ptr[1] = msgpack::object(v.height, o.zone);
	}
};

////////////////////////////////////////////////////////////////////////////////
// cv::Rect_<T>

template<typename T>
struct pack<cv::Rect_<T>> {
	template <typename Stream>
	packer<Stream>& operator()(msgpack::packer<Stream>& o, cv::Rect_<T> const& v) const {

		o.pack_array(4);
		o.pack(v.height);
		o.pack(v.width);
		o.pack(v.x);
		o.pack(v.y);

		return o;
	}
};

template<typename T>
struct convert<cv::Rect_<T>> {
	msgpack::object const& operator()(msgpack::object const& o, cv::Rect_<T> &v) const {
		if (o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }
		if (o.via.array.size != 4) { throw msgpack::type_error(); }

		T height = o.via.array.ptr[0].as<T>();
		T width = o.via.array.ptr[1].as<T>();
		T x = o.via.array.ptr[2].as<T>();
		T y = o.via.array.ptr[3].as<T>();

		v = cv::Rect_<T>(x, y, width, height);
		return o;
	}
};

template <typename T>
struct object_with_zone<cv::Rect_<T>> {
	void operator()(msgpack::object::with_zone& o, cv::Rect_<T> const& v) const {
		o.type = type::ARRAY;
		o.via.array.size = 4;
		o.via.array.ptr = static_cast<msgpack::object*>(
			o.zone.allocate_align(	sizeof(msgpack::object) * o.via.array.size,
									MSGPACK_ZONE_ALIGNOF(msgpack::object)));

		o.via.array.ptr[0] = msgpack::object(v.heigth, o.zone);
		o.via.array.ptr[1] = msgpack::object(v.width, o.zone);
		o.via.array.ptr[2] = msgpack::object(v.x, o.zone);
		o.via.array.ptr[3] = msgpack::object(v.y, o.zone);
	}
};

////////////////////////////////////////////////////////////////////////////////
// cv::Vec

template<typename T, int SIZE>
struct pack<cv::Vec<T, SIZE>> {
	template <typename Stream>
	packer<Stream>& operator()(msgpack::packer<Stream>& o, cv::Vec<T, SIZE> const& v) const {

		o.pack_array(SIZE);
		for (int i = 0; i < SIZE; i++) { o.pack(v[i]); }

		return o;
	}
};

template<typename T, int SIZE>
struct convert<cv::Vec<T, SIZE>> {
	msgpack::object const& operator()(msgpack::object const& o, cv::Vec<T, SIZE> &v) const {
		if (o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }
		if (o.via.array.size != SIZE) { throw msgpack::type_error(); }

		for (int i = 0; i < SIZE; i++) { v[i] = o.via.array.ptr[i].as<T>(); }

		return o;
	}
};

template <typename T, int SIZE>
struct object_with_zone<cv::Vec<T, SIZE>> {
	void operator()(msgpack::object::with_zone& o, cv::Vec<T, SIZE> const& v) const {
		o.type = type::ARRAY;
		o.via.array.size = SIZE;
		o.via.array.ptr = static_cast<msgpack::object*>(
			o.zone.allocate_align(	sizeof(msgpack::object) * o.via.array.size,
									MSGPACK_ZONE_ALIGNOF(msgpack::object)));

		for (int i = 0; i < SIZE; i++) {
			o.via.array.ptr[i] = msgpack::object(v[i], o.zone);
		}
	}
};

////////////////////////////////////////////////////////////////////////////////
// cv::Point_

template<typename T>
struct pack<cv::Point_<T>> {
	template <typename Stream>
	packer<Stream>& operator()(msgpack::packer<Stream>& o, cv::Point_<T> const& p) const {

		o.pack_array(2);
		o.pack(p.x);
		o.pack(p.y);
		return o;
	}
};

template<typename T>
struct convert<cv::Point_<T>> {
	msgpack::object const& operator()(msgpack::object const& o, cv::Point_<T> &p) const {
		if (o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }
		if (o.via.array.size != 2) { throw msgpack::type_error(); }

		p.x = o.via.array.ptr[0].as<T>();
		p.y = o.via.array.ptr[1].as<T>();

		return o;
	}
};

template <typename T>
struct object_with_zone<cv::Point_<T>> {
	void operator()(msgpack::object::with_zone& o, cv::Point_<T> const& p) const {
		o.type = type::ARRAY;
		o.via.array.size = 2;
		o.via.array.ptr = static_cast<msgpack::object*>(
			o.zone.allocate_align(	sizeof(msgpack::object) * o.via.array.size,
									MSGPACK_ZONE_ALIGNOF(msgpack::object)));

		o.via.array.ptr[0] = msgpack::object(p.x, o.zone);
		o.via.array.ptr[1] = msgpack::object(p.y, o.zone);
	}
};

////////////////////////////////////////////////////////////////////////////////
// cv::Point3_

template<typename T>
struct pack<cv::Point3_<T>> {
	template <typename Stream>
	packer<Stream>& operator()(msgpack::packer<Stream>& o, cv::Point3_<T> const& p) const {

		o.pack_array(3);
		o.pack(p.x);
		o.pack(p.y);
		o.pack(p.z);
		return o;
	}
};

template<typename T>
struct convert<cv::Point3_<T>> {
	msgpack::object const& operator()(msgpack::object const& o, cv::Point3_<T> &p) const {
		if (o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }
		if (o.via.array.size != 3) { throw msgpack::type_error(); }

		p.x = o.via.array.ptr[0].as<T>();
		p.y = o.via.array.ptr[1].as<T>();
		p.z = o.via.array.ptr[2].as<T>();

		return o;
	}
};

template <typename T>
struct object_with_zone<cv::Point3_<T>> {
	void operator()(msgpack::object::with_zone& o, cv::Point3_<T> const& p) const {
		o.type = type::ARRAY;
		o.via.array.size = 3;
		o.via.array.ptr = static_cast<msgpack::object*>(
			o.zone.allocate_align(	sizeof(msgpack::object) * o.via.array.size,
									MSGPACK_ZONE_ALIGNOF(msgpack::object)));

		o.via.array.ptr[0] = msgpack::object(p.x, o.zone);
		o.via.array.ptr[1] = msgpack::object(p.y, o.zone);
		o.via.array.ptr[2] = msgpack::object(p.z, o.zone);
	}
};

////////////////////////////////////////////////////////////////////////////////
// cv::Mat

template<>
struct pack<cv::Mat> {
	template <typename Stream>
	packer<Stream>& operator()(msgpack::packer<Stream>& o, cv::Mat const& v) const {
		// TODO: non continuous cv::Mat
		if (!v.isContinuous()) { throw::msgpack::type_error(); }
		o.pack_array(3);
		o.pack(v.type());
		o.pack(v.size());

		auto size = v.total() * v.elemSize();
		o.pack(msgpack::type::raw_ref(reinterpret_cast<char*>(v.data), size));

		return o;
	}
};

template<>
struct convert<cv::Mat> {
	msgpack::object const& operator()(msgpack::object const& o, cv::Mat& v) const {
		if (o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }
		if (o.via.array.size != 3) { throw msgpack::type_error(); }

		int type = o.via.array.ptr[0].as<int>();
		cv::Size size = o.via.array.ptr[1].as<cv::Size>();
		v.create(size, type);

		if (o.via.array.ptr[2].via.bin.size != (v.total() * v.elemSize())) {
			throw msgpack::type_error();
		}

		memcpy(	v.data,
				reinterpret_cast<const uchar*>(o.via.array.ptr[2].via.bin.ptr),
				o.via.array.ptr[2].via.bin.size);

		return o;
	}
};

template <>
struct object_with_zone<cv::Mat> {
	void operator()(msgpack::object::with_zone& o, cv::Mat const& v) const {
		o.type = type::ARRAY;
		o.via.array.size = 3;
		o.via.array.ptr = static_cast<msgpack::object*>(
			o.zone.allocate_align(	sizeof(msgpack::object) * o.via.array.size,
									MSGPACK_ZONE_ALIGNOF(msgpack::object)));

		auto size = v.total() * v.elemSize();
		o.via.array.ptr[0] = msgpack::object(v.type(), o.zone);
		o.via.array.ptr[1] = msgpack::object(v.size(), o.zone);

		// https://github.com/msgpack/msgpack-c/wiki/v2_0_cpp_object#conversion
		// raw_ref not copied to zone (is this a problem?)
		o.via.array.ptr[2] = msgpack::object(
			msgpack::type::raw_ref(reinterpret_cast<char*>(v.data), static_cast<uint32_t>(size)),
			o.zone);
	}
};

////////////////////////////////////////////////////////////////////////////////
// Eigen::Matrix<>

template <typename T, int X, int Y>
struct pack<Eigen::Matrix<T, X, Y>> {
	template <typename Stream>
	packer<Stream>& operator()(msgpack::packer<Stream>& o, Eigen::Matrix<T, X, Y> const& v) const {

		o.pack_array(X*Y);
		for (int i = 0; i < X*Y; i++) { o.pack(v.data()[i]); }

		return o;
	}
};

template<typename T, int X, int Y>
struct convert<Eigen::Matrix<T, X, Y>> {
	msgpack::object const& operator()(msgpack::object const& o, Eigen::Matrix<T, X, Y> &v) const {
		if (o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }
		if (o.via.array.size != X*Y) { throw msgpack::type_error(); }

		for (int i = 0; i < X*Y; i++) { v.data()[i] = o.via.array.ptr[i].as<T>(); }

		return o;
	}
};

template <typename T, int X, int Y>
struct object_with_zone<Eigen::Matrix<T, X, Y>> {
	void operator()(msgpack::object::with_zone& o, Eigen::Matrix<T, X, Y> const& v) const {
		o.type = type::ARRAY;
		o.via.array.size = X*Y;
		o.via.array.ptr = static_cast<msgpack::object*>(
			o.zone.allocate_align(	sizeof(msgpack::object) * o.via.array.size,
									MSGPACK_ZONE_ALIGNOF(msgpack::object)));

		for (int i = 0; i < X*Y; i++) {
			o.via.array.ptr[i] = msgpack::object(v.data()[i], o.zone);
		}
	}
};

}
}
}

#endif
