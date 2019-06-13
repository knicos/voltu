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

	if (options.find("intrinsic") != options.end()) {
		ftl::calibration::intrinsic(options);
	} else if (options.find("stereo") != options.end()) {
		ftl::calibration::stereo(options);
	} else if (options.find("align") != options.end()) {
		ftl::calibration::align(options);
	} else {
		LOG(ERROR) << "Must have one of: --intrinsic --stereo or --align";
	}

	return 0;
}
