#include <loguru.hpp>
#include <ftl/configuration.hpp>

#include "lens.hpp"
#include "stereo.hpp"
#include "align.hpp"

int main(int argc, char **argv) {
	loguru::g_preamble_date = false;
	loguru::g_preamble_uptime = false;
	loguru::g_preamble_thread = false;
	loguru::init(argc, argv, "--verbosity");
	argc--;
	argv++;

	// Process Arguments
	auto options = ftl::config::read_options(&argv, &argc);

	ftl::calibration::intrinsic(options);

	return 0;
}
