#include "catch.hpp"
#include <ftl/net_configurable.hpp>
#include <ftl/slave.hpp>

using ftl::NetConfigurable;

SCENARIO( "NetConfigurable::set()" ) {
    GIVEN( "valid peer UUID, URI and Master" ) {
        // Set up Master
        nlohmann::json json = {{"$id", "root"}, {"test", {{"listen", "tcp://localhost:7077"}}}}; // Check what values are needed
        ftl::Configurable *root;
        root = new ftl::Configurable(json);
        ftl::net::Universe *net = ftl::config::create<ftl::net::Universe>(root, std::string("test"));
        net->start();
        ftl::ctrl::Master *controller = new ftl::ctrl::Master(root, net);
        
        // Set up a slave, then call getSlaves() to get the UUID string
        nlohmann::json jsonSlave = {{"$id", "slave"}, {"test", {{"peers", {"tcp://localhost:7077"}}}}};
        ftl::Configurable *rootSlave;
        rootSlave = new ftl::Configurable(jsonSlave);
        ftl::net::Universe *netSlave = ftl::config::create<ftl::net::Universe>(rootSlave, std::string("test"));
        ftl::ctrl::Slave slave(netSlave, rootSlave);
        netSlave->start();
        netSlave->waitConnections();
        net->waitConnections();

        auto slaves = controller->getSlaves();
        REQUIRE( slaves.size() == 1 );

        ftl::UUID peer = ftl::UUID(slaves[0]["id"].get<std::string>());
        const std::string suri = "slave_test";
        nlohmann::json jsonTest = {{"$id", "slave_test"}, {"test", {{"peers", {"tcp://localhost:7077"}}}}};
        NetConfigurable nc(peer, suri, *controller, jsonTest);
        nc.set("test_value", 5);
        REQUIRE( nc.get<int>("test_value") == 5 );
    }

    // invalid peer UUID

    // invalid URI

    // null Master
}