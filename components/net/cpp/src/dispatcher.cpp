//#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <loguru.hpp>
#include <ftl/net/dispatcher.hpp>
#include <ftl/net/peer.hpp>
#include <iostream>

using ftl::net::Peer;
using ftl::net::Dispatcher;
using std::vector;
using std::string;
using std::optional;

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

//void ftl::net::Dispatcher::dispatch(Peer &s, const std::string &msg) {
	//std::cout << "Received dispatch : " << hexStr(msg) << std::endl;
//    auto unpacked = msgpack::unpack(msg.data(), msg.size());
//    dispatch(s, unpacked.get());
//}

vector<string> Dispatcher::getBindings() const {
	vector<string> res;
	for (auto x : funcs_) {
		res.push_back(x.first);
	}
	return res;
}

void ftl::net::Dispatcher::dispatch(Peer &s, const msgpack::object &msg) {
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

void ftl::net::Dispatcher::dispatch_call(Peer &s, const msgpack::object &msg) {
    call_t the_call;
    
    try {
    	msg.convert(the_call);
    } catch(...) {
    	LOG(ERROR) << "Bad message format";
    	return;
    }

    // TODO: proper validation of protocol (and responding to it)
    auto &&type = std::get<0>(the_call);
    auto &&id = std::get<1>(the_call);
	auto &&name = std::get<2>(the_call);
	auto &&args = std::get<3>(the_call);
    // assert(type == 0);
    
    if (type == 1) {
    	DLOG(INFO) << "RPC return for " << id;
    	s._dispatchResponse(id, args);
    } else if (type == 0) {
		DLOG(INFO) << "RPC " << name << "() <- " << s.getURI();

		auto func = _locateHandler(name);

		if (func) {
			DLOG(INFO) << "Found binding for " << name;
		    try {
		        auto result = (*func)(args); //->get();
		        s._sendResponse(id, result->get());
		        /*response_t res_obj = std::make_tuple(1,id,msgpack::object(),result->get());
				std::stringstream buf;
				msgpack::pack(buf, res_obj);			
				s.send("__return__", buf.str());*/
			} catch (const std::exception &e) {
				//throw;
				LOG(ERROR) << "Exception when attempting to call RPC (" << e.what() << ")";
		        /*response_t res_obj = std::make_tuple(1,id,msgpack::object(e.what()),msgpack::object());
				std::stringstream buf;
				msgpack::pack(buf, res_obj);			
				s.send("__return__", buf.str());*/
			} catch (int e) {
				//throw;
				LOG(ERROR) << "Exception when attempting to call RPC (" << e << ")";
		        /*response_t res_obj = std::make_tuple(1,id,msgpack::object(e),msgpack::object());
				std::stringstream buf;
				msgpack::pack(buf, res_obj);			
				s.send("__return__", buf.str());*/
			}
		} else {
			LOG(WARNING) << "No binding found for " << name;
		}
	} else {
		// TODO(nick) Some error
		LOG(ERROR) << "Unrecognised message type";
	}
}

optional<Dispatcher::adaptor_type> ftl::net::Dispatcher::_locateHandler(const std::string &name) const {
	auto it_func = funcs_.find(name);
	if (it_func == end(funcs_)) {
		if (parent_ != nullptr) {
			return parent_->_locateHandler(name);
		} else {
			return {};
		}
	} else {
		return it_func->second;
	}
}

void ftl::net::Dispatcher::dispatch_notification(Peer &s, msgpack::object const &msg) {
    notification_t the_call;
    msg.convert(the_call);

    // TODO: proper validation of protocol (and responding to it)
    // auto &&type = std::get<0>(the_call);
    // assert(type == static_cast<uint8_t>(request_type::notification));

    auto &&name = std::get<1>(the_call);
    auto &&args = std::get<2>(the_call);

    auto binding = _locateHandler(name);
	//LOG(INFO) << "NOTIFICATION " << name << "() <- " << s.getURI();

    if (binding) {
        try {
            auto result = (*binding)(args);
        } catch (const int &e) {
			LOG(ERROR) << "Exception in bound function";
			throw &e;
		} catch (const std::exception &e) {
			LOG(ERROR) << "Exception for '" << name << "' - " << e.what();
		}
    } else {
    	LOG(ERROR) << "Missing handler for incoming message (" << name << ")";
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
    	LOG(FATAL) << "RPC non unique binding for '" << func << "'";
        throw -1;
    }
}

