/* Taken from rpclib */

#pragma once

#ifndef FUNC_TRAITS_H_HWIWA6G0
#define FUNC_TRAITS_H_HWIWA6G0

#include <type_traits>
#include <tuple>

namespace ftl {

namespace net {
class Peer;
}

namespace internal {

template<typename T>
using invoke = typename T::type;

template<typename T, T I>
struct constant : std::integral_constant<T, I> {};

template<bool B>
using bool_ = constant<bool, B>;

using true_ = bool_<true>;

using false_ = bool_<false>;

template <int N>
using is_zero = invoke<std::conditional<(N == 0), true_, false_>>;

template <int N, typename... Ts>
using nth_type = invoke<std::tuple_element<N, std::tuple<Ts...>>>;

namespace tags {

// tags for the function traits, used for tag dispatching
struct zero_arg {};
struct nonzero_arg {};
struct void_result {};
struct nonvoid_result {};

template <int N> struct arg_count_trait { typedef nonzero_arg type; };

template <> struct arg_count_trait<0> { typedef zero_arg type; };

template <typename T> struct result_trait { typedef nonvoid_result type; };

template <> struct result_trait<void> { typedef void_result type; };
}

//! \brief Provides a small function traits implementation that
//! works with a reasonably large set of functors.
template <typename T>
struct func_traits : func_traits<decltype(&T::operator())> {};

template <typename C, typename R, typename... Args>
struct func_traits<R (C::*)(Args...)> : func_traits<R (*)(Args...)> {};

template <typename C, typename R, typename... Args>
struct func_traits<R (C::*)(Args...) const> : func_traits<R (*)(Args...)> {};

template <typename R, typename... Args> struct func_traits<R (*)(ftl::net::Peer &,Args...)> {
    using result_type = R;
    using arg_count = std::integral_constant<std::size_t, sizeof...(Args)>;
    using args_type = std::tuple<typename std::decay<Args>::type...>;
};

template <typename R, typename... Args> struct func_traits<R (*)(Args...)> {
    using result_type = R;
    using arg_count = std::integral_constant<std::size_t, sizeof...(Args)>;
    using args_type = std::tuple<typename std::decay<Args>::type...>;
};

//template <typename T>
//auto bindThis(F f, T t) { return [f,t]()t.f(42, std::forward<decltype(arg)>(arg)); }

template <typename T>
struct func_kind_info : func_kind_info<decltype(&T::operator())> {};

template <typename C, typename R, typename... Args>
struct func_kind_info<R (C::*)(Args...)> : func_kind_info<R (*)(Args...)> {};

//template <typename R, typename... Args>
//struct func_kind_info<std::_Bind<R(Args...)>> : func_kind_info<R(*)(Args...)> {};

template <typename C, typename R, typename... Args>
struct func_kind_info<R (C::*)(Args...) const>
    : func_kind_info<R (*)(Args...)> {};

template <typename R, typename... Args> struct func_kind_info<R (*)(ftl::net::Peer &,Args...)> {
    typedef typename tags::arg_count_trait<sizeof...(Args)>::type args_kind;
    typedef typename tags::result_trait<R>::type result_kind;
	typedef true_ has_peer;
};

template <typename R, typename... Args> struct func_kind_info<R (*)(Args...)> {
    typedef typename tags::arg_count_trait<sizeof...(Args)>::type args_kind;
    typedef typename tags::result_trait<R>::type result_kind;
	typedef false_ has_peer;
};

template <typename F> using is_zero_arg = is_zero<func_traits<F>::arg_count>;

template <typename F>
using is_single_arg =
    invoke<std::conditional<func_traits<F>::arg_count == 1, true_, false_>>;

template <typename F>
using is_void_result = std::is_void<typename func_traits<F>::result_type>;
}
}

#endif /* end of include guard: FUNC_TRAITS_H_HWIWA6G0 */

