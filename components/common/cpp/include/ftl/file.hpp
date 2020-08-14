#ifndef _FTL_FILES_HPP_
#define _FTL_FILES_HPP_

#if defined(__GNUC__) && __GNUC__ < 8
#include <experimental/filesystem>
namespace std {
namespace filesystem = experimental::filesystem;
}

#else
#include <filesystem>
#endif

namespace ftl {
namespace file {

std::filesystem::path home_dir();
std::filesystem::path config_dir();

bool is_file(const std::filesystem::path &path);

}
}

#endif