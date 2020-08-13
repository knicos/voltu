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

path config_dir(const path& subdir) {
	#if defined(_MSC_VER)
	return path(std::getenv("APPDATA")) / subdir;
	#elif defined(__GNUC__)
	return home_dir() / ".config";
	#else
	static_assert(false, "unsupported compiler");
	#endif
}

path config_dir() {
	return config_dir(path());
}

bool is_file(const path& p) {
	return std::filesystem::is_regular_file(p);
}

}
}
