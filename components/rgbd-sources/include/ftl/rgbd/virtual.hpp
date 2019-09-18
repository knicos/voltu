#ifndef _FTL_RGBD_VIRTUAL_HPP_
#define _FTL_RGBD_VIRTUAL_HPP_

#include <ftl/rgbd/source.hpp>

namespace ftl {
namespace rgbd {

class VirtualSource : public ftl::rgbd::Source {
    public:
    explicit VirtualSource(ftl::config::json_t &cfg);
	~VirtualSource();

    /**
	 * Write frames into source buffers from an external renderer. Virtual
	 * sources do not have an internal generator of frames but instead have
	 * their data provided from an external rendering class. This function only
	 * works when there is no internal generator.
	 */
    void write(int64_t ts, ftl::rgbd::Frame &frame, cudaStream_t stream=0);
};

}
}

#endif  // _FTL_RGBD_VIRTUAL_HPP_
