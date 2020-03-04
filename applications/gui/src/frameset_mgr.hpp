#ifndef _FTL_GUI_FRAMESET_MANAGER_HPP_
#define _FTL_GUI_FRAMESET_MANAGER_HPP_

#include <string>

namespace ftl {
namespace gui {

/**
 * Given a stream URI, allocate a frameset number to that stream.
 */
int mapToFrameset(const std::string &uri);

}
}

#endif
