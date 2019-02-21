#ifndef _FTL_NET_HANDLERS_HPP_
#define _FTL_NET_HANDLERS_HPP_

#include <functional>

namespace ftl {
namespace net {

class Socket;

typedef std::function<void(int, std::string&)> sockdatahandler_t;
typedef std::function<void(int)> sockerrorhandler_t;
typedef std::function<void()> sockconnecthandler_t;
typedef std::function<void(int)> sockdisconnecthandler_t;

typedef std::function<void(Socket&, int, std::string&)> datahandler_t;
typedef std::function<void(Socket&, int)> errorhandler_t;
typedef std::function<void(Socket&)> connecthandler_t;
typedef std::function<void(Socket&)> disconnecthandler_t;

};
};

#endif // _FTL_NET_HANDLERS_HPP_

