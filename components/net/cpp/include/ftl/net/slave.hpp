#ifndef _FTL_NET_SLAVE_HPP_
#define _FTL_NET_SLAVE_HPP_

#include <ftl/net/universe.hpp>
#include <ftl/configurable.hpp>

namespace ftl {
namespace net {

class Slave {
	public:
	Slave(Universe *, ftl::Configurable *);
	~Slave();
};

}
}

#endif  // _FTL_NET_SLAVE_HPP_
