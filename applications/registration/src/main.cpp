#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <string>

#include "manual.hpp"

using std::string;

int main(int argc, char **argv) {
	auto root = ftl::configure(argc, argv, "registration_default");

	// TODO
	ftl::registration::manual(root);
	if (root->value("mode", string("manual") == "manual")) {
		//ftl::registration::manual(root);
	}

	return 0;
}
