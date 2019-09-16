#ifndef _FTL_TRAITS_HPP_
#define _FTL_TRAITS_HPP_

#include <opencv2/core.hpp>
#include <ftl/cuda_util.hpp>

namespace ftl {
namespace traits {

template <typename T>
struct AlwaysFalse : std::false_type {};

template <typename T> struct OpenCVType {
	static_assert(AlwaysFalse<T>::value, "Not a valid format type");
};
template <> struct OpenCVType<uchar> { static const int value = CV_8UC1; };
template <> struct OpenCVType<uchar2> { static const int value = CV_8UC2; };
template <> struct OpenCVType<uchar3> { static const int value = CV_8UC3; };
template <> struct OpenCVType<uchar4> { static const int value = CV_8UC4; };
template <> struct OpenCVType<char> { static const int value = CV_8SC1; };
template <> struct OpenCVType<char2> { static const int value = CV_8SC2; };
template <> struct OpenCVType<char3> { static const int value = CV_8SC3; };
template <> struct OpenCVType<char4> { static const int value = CV_8SC4; };
template <> struct OpenCVType<ushort> { static const int value = CV_16UC1; };
template <> struct OpenCVType<ushort2> { static const int value = CV_16UC2; };
template <> struct OpenCVType<ushort3> { static const int value = CV_16UC3; };
template <> struct OpenCVType<ushort4> { static const int value = CV_16UC4; };
template <> struct OpenCVType<short> { static const int value = CV_16SC1; };
template <> struct OpenCVType<short2> { static const int value = CV_16SC2; };
template <> struct OpenCVType<short3> { static const int value = CV_16SC3; };
template <> struct OpenCVType<short4> { static const int value = CV_16SC4; };
template <> struct OpenCVType<int> { static const int value = CV_32SC1; };
template <> struct OpenCVType<int2> { static const int value = CV_32SC2; };
template <> struct OpenCVType<int3> { static const int value = CV_32SC3; };
template <> struct OpenCVType<int4> { static const int value = CV_32SC4; };
template <> struct OpenCVType<float> { static const int value = CV_32FC1; };
template <> struct OpenCVType<float2> { static const int value = CV_32FC2; };
template <> struct OpenCVType<float3> { static const int value = CV_32FC3; };
template <> struct OpenCVType<float4> { static const int value = CV_32FC4; };

}
}

#endif  // _FTL_TRAITS_HPP_
