/**
 * @file file.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

#include "ftl/file.hpp"

#include <cstdlib>

using std::filesystem::path;

namespace ftl {
namespace file {

path home_dir() {
	char* home;
	#if defined(_MSC_VER)
	home = std::getenv("HOMEPATH");
	#elif defined(__GNUC__)
	home = std::getenv("HOME");
	#else
	static_assert(false, "unsupported compiler");
	#endif
	return std::filesystem::absolute(path(home));
}

path config_dir() {
	#if defined(_MSC_VER)
	return path(std::getenv("APPDATA")) / "ftl";
	#elif defined(__GNUC__)
	return home_dir() / ".config" / "ftl";
	#else
	static_assert(false, "unsupported compiler");
	#endif
}

bool is_file(const path& p) {
	return std::filesystem::is_regular_file(p);
}

}
}
