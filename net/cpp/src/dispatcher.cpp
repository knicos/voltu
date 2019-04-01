#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#include <ftl/net/dispatcher.hpp>
#include <ftl/net/socket.hpp>
#include <iostream>

using ftl::net::Socket;

/*static std::string hexStr(const std::string &s)
{
	const char *data = s.data();
	int len = s.size();
    std::stringstream ss;
    ss << std::hex;
    for(int i=0;i<len;++i)
        ss << std::setw(2) << std::setfill('0') << (int)data[i];
    return ss.str();
}*/

void ftl::net::Dispatcher::dispatch(Socket &s, const std::string &msg) {
	//std::cout << "Received dispatch : " << hexStr(msg) << std::endl;
    auto unpacked = msgpack::unpack(msg.data(), msg.size());
    dispatch(s, unpacked.get());
}

void ftl::net::Dispatcher::dispatch(Socket &s, const msgpack::object &msg) {
    switch (msg.via.array.size) {
    case 3:
        dispatch_notification(s, msg); break;
    case 4:
        dispatch_call(s, msg); break;
    default:
    	LOG(ERROR) << "Unrecognised msgpack : " << msg.via.array.size;
        return;
    }
}

void ftl::net::Dispatcher::dispatch_call(Socket &s, const msgpack::object &msg) {
    call_t the_call;
    msg.convert(the_call);

    // TODO: proper validation of protocol (and responding to it)
    // auto &&type = std::get<0>(the_call);
    // assert(type == 0);

    auto &&id = std::get<1>(the_call);
    auto &&name = std::get<2>(the_call);
    auto &&args = std::get<3>(the_call);
    
    LOG(INFO) << "RPC " << name << "() <- " << s.getURI();

    auto it_func = funcs_.find(name);

    if (it_func != end(funcs_)) {
        try {
            auto result = (it_func->second)(args); //->get();
            response_t res_obj = std::make_tuple(1,id,msgpack::object(),result->get());
			std::stringstream buf;
			msgpack::pack(buf, res_obj);			
			s.send(FTL_PROTOCOL_RPCRETURN, buf.str());
		} catch (const std::exception &e) {
			//throw;
			//LOG(ERROR) << "Exception when attempting to call RPC (" << e << ")";
            response_t res_obj = std::make_tuple(1,id,msgpack::object(e.what()),msgpack::object());
			std::stringstream buf;
			msgpack::pack(buf, res_obj);			
			s.send(FTL_PROTOCOL_RPCRETURN, buf.str());
		} catch (int e) {
			//throw;
			//LOG(ERROR) << "Exception when attempting to call RPC (" << e << ")";
            response_t res_obj = std::make_tuple(1,id,msgpack::object(e),msgpack::object());
			std::stringstream buf;
			msgpack::pack(buf, res_obj);			
			s.send(FTL_PROTOCOL_RPCRETURN, buf.str());
		}
    }
}

void ftl::net::Dispatcher::dispatch_notification(Socket &s, msgpack::object const &msg) {
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
        } catch (int e) {
			throw e;
		}
    }
}

void ftl::net::Dispatcher::enforce_arg_count(std::string const &func, std::size_t found,
                                   std::size_t expected) {
    if (found != expected) {
    	LOG(FATAL) << "RPC argument missmatch - " << found << " != " << expected;
        throw -1;
    }
}

void ftl::net::Dispatcher::enforce_unique_name(std::string const &func) {
    auto pos = funcs_.find(func);
    if (pos != end(funcs_)) {
        throw -1;
    }
}

