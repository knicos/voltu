#ifndef _FTL_CTRL_SLAVE_HPP_
#define _FTL_CTRL_SLAVE_HPP_

#include <ftl/net/universe.hpp>
#include <ftl/configurable.hpp>

namespace ftl {
namespace ctrl {

class Slave {
	public:
	Slave(ftl::net::Universe *, ftl::Configurable *);
	~Slave();
};

}
}

#endif  // _FTL_CTRL_SLAVE_HPP_
