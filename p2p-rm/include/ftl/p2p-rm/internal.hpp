#ifndef _FTL_P2P_RM_INTERNAL_HPP_
#define _FTL_P2P_RM_INTERNAL_HPP_

namespace ftl {
namespace rm {

class Blob;

enum flags_t : uint32_t {
	FLAG_INTEGER = 1,
	FLAG_SIGNED = 2,
	FLAG_TRIVIAL = 4
};

ftl::rm::Blob *_lookup(const char *uri);
ftl::rm::Blob *_create(const char *uri, char *addr, size_t size, size_t count, flags_t flags, const std::string &tname);

}; // namespace rm
}; // namespace ftl

#endif // _FTL_P2P_RM_INTERNAL_HPP_

