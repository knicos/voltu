#include "catch.hpp"
#include <ftl/net_configurable.hpp>
#include <ftl/master.hpp>

using ftl::NetConfigurable;

SCENARIO( "NetConfigurable::set()" ) {
    GIVEN( "valid peer UUID, URI and Master" ) {
        // Set up Master
        nlohmann::json json = nlohmann::json{{"$id", "root"}, {"test", {{"listen", "tcp://localhost:7077"}}}}; // Check what values are needed
        ftl::Configurable *root;
        root = new ftl::Configurable(json);
        ftl::net::Universe *net = ftl::config::create<ftl::net::Universe>(root, std::string("test"));
        net->start();
        ftl::ctrl::Master *controller = new ftl::ctrl::Master(root, net);
        
        // Set up a slave, then call getControllers() to get the UUID string
        nlohmann::json jsonSlave = nlohmann::json{{"$id", "slave"}, {"test", {{"peers", {"tcp://localhost:7077"}}}}};
        ftl::Configurable *rootSlave;
        rootSlave = new ftl::Configurable(jsonSlave);
        ftl::net::Universe *netSlave = ftl::config::create<ftl::net::Universe>(rootSlave, std::string("test"));
        ftl::ctrl::Master ctrl(rootSlave, netSlave);
        netSlave->start();
        netSlave->waitConnections();
        net->waitConnections();

        auto controllers = controller->getControllers();
        REQUIRE( controllers.size() == 1 );

        ftl::UUID peer = ftl::UUID(controllers[0]["id"].get<std::string>());
        const std::string suri = "slave_test";
        nlohmann::json jsonTest = nlohmann::json{{"$id", "slave_test"}, {"test", {{"peers", {"tcp://localhost:7077"}}}}};
        NetConfigurable nc(peer, suri, *controller, jsonTest);
        nc.set("test_value", 5);
        REQUIRE( nc.get<int>("test_value") == 5 );

        delete controller;
    }

    // invalid peer UUID

    // invalid URI

    // null Master
}