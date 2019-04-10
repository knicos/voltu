#ifndef _FTL_NET_DISPATCHER_HPP_
#define _FTL_NET_DISPATCHER_HPP_

#include <ftl/net/func_traits.hpp>

#ifdef _MSC_VER
#include <msgpack_optional.hpp>
#endif

#include <msgpack.hpp>
#include <memory>
#include <tuple>
#include <functional>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <optional>

namespace ftl {

namespace internal {
	//! \brief Calls a functor with argument provided directly
	template <typename Functor, typename Arg>
	auto call(Functor f, Arg &&arg)
		-> decltype(f(std::forward<Arg>(arg)))
	{
		return f(std::forward<Arg>(arg));
	}

	template <typename Functor, typename... Args, std::size_t... I>
	decltype(auto) call_helper(Functor func, std::tuple<Args...> &&params,
		                       std::index_sequence<I...>) {
		return func(std::get<I>(params)...);
	}

	//! \brief Calls a functor with arguments provided as a tuple
	template <typename Functor, typename... Args>
	decltype(auto) call(Functor f, std::tuple<Args...> &args) {
		return call_helper(f, std::forward<std::tuple<Args...>>(args),
		                   std::index_sequence_for<Args...>{});
	}
}

namespace net {
class Peer;

class Dispatcher {
	public:
	explicit Dispatcher(Dispatcher *parent=nullptr) : parent_(parent) {}
	
	//void dispatch(Peer &, const std::string &msg);
	void dispatch(Peer &, const msgpack::object &msg);
	
	template <typename F>
	void bind(std::string const &name, F func,
		                  ftl::internal::tags::void_result const &,
		                  ftl::internal::tags::zero_arg const &) {
		enforce_unique_name(name);
		funcs_.insert(
		    std::make_pair(name, [func, name](msgpack::object const &args) {
		        enforce_arg_count(name, 0, args.via.array.size);
		        func();
		        return std::make_unique<msgpack::object_handle>();
		    }));
	}

	template <typename F>
	void bind(std::string const &name, F func,
		                  ftl::internal::tags::void_result const &,
		                  ftl::internal::tags::nonzero_arg const &) {
		using ftl::internal::func_traits;
		using args_type = typename func_traits<F>::args_type;

		enforce_unique_name(name);
		funcs_.insert(
		    std::make_pair(name, [func, name](msgpack::object const &args) {
		        constexpr int args_count = std::tuple_size<args_type>::value;
		        enforce_arg_count(name, args_count, args.via.array.size);
		        args_type args_real;
		        args.convert(args_real);
		        ftl::internal::call(func, args_real);
		        return std::make_unique<msgpack::object_handle>();
		    }));
	}

	template <typename F>
	void bind(std::string const &name, F func,
		                  ftl::internal::tags::nonvoid_result const &,
		                  ftl::internal::tags::zero_arg const &) {
		using ftl::internal::func_traits;

		enforce_unique_name(name);
		funcs_.insert(std::make_pair(name, [func,
		                                    name](msgpack::object const &args) {
		    enforce_arg_count(name, 0, args.via.array.size);
		    auto z = std::make_unique<msgpack::zone>();
		    auto result = msgpack::object(func(), *z);
		    return std::make_unique<msgpack::object_handle>(result, std::move(z));
		}));
	}

	template <typename F>
	void bind(std::string const &name, F func,
		                  ftl::internal::tags::nonvoid_result const &,
		                  ftl::internal::tags::nonzero_arg const &) {
		using ftl::internal::func_traits;
		using args_type = typename func_traits<F>::args_type;

		enforce_unique_name(name);
		funcs_.insert(std::make_pair(name, [func,
		                                    name](msgpack::object const &args) {
		    constexpr int args_count = std::tuple_size<args_type>::value;
		    enforce_arg_count(name, args_count, args.via.array.size);
		    args_type args_real;
		    args.convert(args_real);
		    auto z = std::make_unique<msgpack::zone>();
		    auto result = msgpack::object(ftl::internal::call(func, args_real), *z);
		    return std::make_unique<msgpack::object_handle>(result, std::move(z));
		}));
	}
	
	std::vector<std::string> getBindings() const;
	
	using adaptor_type = std::function<std::unique_ptr<msgpack::object_handle>(
        msgpack::object const &)>;

    //! \brief This is the type of messages as per the msgpack-rpc spec.
    using call_t = std::tuple<int8_t, uint32_t, std::string, msgpack::object>;

    //! \brief This is the type of notification messages.
    using notification_t = std::tuple<int8_t, std::string, msgpack::object>;
    
    using response_t =
        std::tuple<uint32_t, uint32_t, msgpack::object, msgpack::object>;
	
	private:
	Dispatcher *parent_;
	std::unordered_map<std::string, adaptor_type> funcs_;
	
	std::optional<adaptor_type> _locateHandler(const std::string &name) const;
	
	static void enforce_arg_count(std::string const &func, std::size_t found,
                                  std::size_t expected);

    void enforce_unique_name(std::string const &func);

	void dispatch_call(Peer &, const msgpack::object &msg);
	void dispatch_notification(Peer &, msgpack::object const &msg);
};

}
}

#endif // _FTL_NET_DISPATCHER_HPP_

