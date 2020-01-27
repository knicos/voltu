#ifndef _FTL_MSGPACK_HPP_
#define _FTL_MSGPACK_HPP_

#ifdef _MSC_VER
#include "msgpack_optional.hpp"
#endif

#include <msgpack.hpp>
#include <opencv2/core/mat.hpp>

namespace msgpack {
MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
namespace adaptor {

////////////////////////////////////////////////////////////////////////////////
// cv::Size

template<>
struct pack<cv::Size> {
	template <typename Stream>
	packer<Stream>& operator()(msgpack::packer<Stream>& o, cv::Size const& v) const {

		o.pack_array(2);
		o.pack(v.width);
		o.pack(v.height);

		return o;
	}
};

template<>
struct convert<cv::Size> {
	msgpack::object const& operator()(msgpack::object const& o, cv::Size& v) const {
		if (o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }
		if (o.via.array.size != 2) { throw msgpack::type_error(); }
		
		int width = o.via.array.ptr[0].as<int>();
		int height = o.via.array.ptr[1].as<int>();
		v = cv::Size(width, height);
		return o;
	}
};

template <>
struct object_with_zone<cv::Size> {
	void operator()(msgpack::object::with_zone& o, cv::Size const& v) const {
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
			msgpack::type::raw_ref(reinterpret_cast<char*>(v.data), size),
			o.zone);
	}
};


}
}
}

#endif
