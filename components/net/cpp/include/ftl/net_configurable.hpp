#pragma once
#ifndef _FTL_NETCONFIGURABLE_HPP_
#define _FTL_NETCONFIGURABLE_HPP_

#include <ftl/configurable.hpp>
#include <ftl/master.hpp>

namespace ftl {

    class NetConfigurable : public ftl::Configurable {
    public:
	NetConfigurable(ftl::UUID peer, const std::string &suri, ftl::ctrl::Master &ctrl, ftl::config::json_t &config);
	~NetConfigurable();

	protected:
	void inject(const std::string &name, nlohmann::json &value);

    private:
	ftl::UUID peer;
	const std::string suri;
	ftl::ctrl::Master &ctrl;
    };

}

#endif // _FTL_NETCONFIGURABLE_HPP_
