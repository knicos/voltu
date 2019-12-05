#include <ftl/net_configurable.hpp>

#include <string>

ftl::NetConfigurable::NetConfigurable(ftl::UUID peer, const std::string &suri, ftl::ctrl::Master &ctrl, ftl::config::json_t &config) : ftl::Configurable(config), peer(peer), suri(suri), ctrl(ctrl) {
}

ftl::NetConfigurable::~NetConfigurable(){}

void ftl::NetConfigurable::inject(const std::string &name, nlohmann::json &value) {
    ctrl.set(peer, suri + std::string("/") + name, value);
}

void ftl::NetConfigurable::refresh() {
    (*config_) = ctrl.get(peer, suri);
}
