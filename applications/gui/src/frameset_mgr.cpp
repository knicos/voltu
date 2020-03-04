#include "frameset_mgr.hpp"
#include <ftl/uri.hpp>

static int frameset_counter = 0;

int ftl::gui::mapToFrameset(const std::string &uri) {
    ftl::URI u(uri);
    return frameset_counter++;
}
