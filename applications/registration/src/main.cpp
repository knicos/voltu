#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <string>

#include "manual.hpp"
#include "aruco.hpp"

using std::string;

int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "registration_default");

	string mode = root->value("mode", string("manual"));
	if (mode == "manual") {
		ftl::registration::manual(root);
	} else if (mode == "aruco") {
		ftl::registration::aruco(root);
	}

	return 0;
}
