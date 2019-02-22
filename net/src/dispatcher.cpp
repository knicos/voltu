#include <ftl/net/dispatcher.hpp>

void ftl::net::Dispatcher::dispatch(const std::string &msg) {
    auto unpacked = msgpack::unpack(msg.data(), msg.size());
    dispatch(unpacked.get());
}

void ftl::net::Dispatcher::dispatch(const msgpack::object &msg) {
    switch (msg.via.array.size) {
    case 3:
        dispatch_notification(msg);
    case 4:
        dispatch_call(msg);
    default:
        return;
    }
}

void ftl::net::Dispatcher::dispatch_call(const msgpack::object &msg) {
    call_t the_call;
    msg.convert(the_call);

    // TODO: proper validation of protocol (and responding to it)
    // auto &&type = std::get<0>(the_call);
    // assert(type == 0);

   // auto &&id = std::get<1>(the_call);
    auto &&name = std::get<2>(the_call);
    auto &&args = std::get<3>(the_call);

    auto it_func = funcs_.find(name);

    if (it_func != end(funcs_)) {
        try {
            auto result = (it_func->second)(args);
            // TODO SEND RESULTS
        } catch (...) {
			throw;
		}
    }
}

void ftl::net::Dispatcher::dispatch_notification(msgpack::object const &msg) {
    notification_t the_call;
    msg.convert(the_call);

    // TODO: proper validation of protocol (and responding to it)
    // auto &&type = std::get<0>(the_call);
    // assert(type == static_cast<uint8_t>(request_type::notification));

    auto &&name = std::get<1>(the_call);
    auto &&args = std::get<2>(the_call);

    auto it_func = funcs_.find(name);

    if (it_func != end(funcs_)) {
        try {
            auto result = (it_func->second)(args);
        } catch (...) {
			throw;
		}
    }
}

void ftl::net::Dispatcher::enforce_arg_count(std::string const &func, std::size_t found,
                                   std::size_t expected) {
    if (found != expected) {
        throw;
    }
}

void ftl::net::Dispatcher::enforce_unique_name(std::string const &func) {
    auto pos = funcs_.find(func);
    if (pos != end(funcs_)) {
        throw;
    }
}

