#ifndef _FTL_RGBD_DETAIL_SOURCE_HPP_
#define _FTL_RGBD_DETAIL_SOURCE_HPP_

#include <Eigen/Eigen>
#include <ftl/cuda_util.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/rgbd/frame.hpp>

namespace ftl{
namespace rgbd {

class Source;

/**
 * Base class for source device implementations. Each device provides a capture
 * and retrieve functionality. `capture` is called with a high resolution timer
 * at a precise timestamp to ensure synchronisation. In another thread the
 * `retrieve` function is called after `capture` to download any data into the
 * provided frame object. The frame object is then dispatched for further
 * processing, such as disparity calculation, or is discarded if a previous
 * processing dispatch is still on going.
 * 
 * @see ftl::rgbd::Group
 */
class BaseSourceImpl {
	public:
	// TODO: Remove this
	friend class ftl::rgbd::Source;

	public:
	explicit BaseSourceImpl(ftl::rgbd::Source *host) : capabilities_(0), host_(host) { }
	virtual ~BaseSourceImpl() {}

	/**
	 * Perform hardware data capture. This should be low latency (<1ms).
	 */
	virtual bool capture(int64_t ts)=0;

	/**
	 * Perform slow IO operation to get the data into the given frame object.
	 * This can take up to 1 fps (eg. ~40ms), but should be faster. It occurs
	 * in a different thread to the `capture` call but will never occur at the
	 * same time as `capture`. If `capture` fails then this will not be called.
	 */
	virtual bool retrieve(ftl::rgbd::Frame &frame)=0;

	/**
	 * Is the source ready to capture and retrieve?
	 */
	virtual bool isReady() { return false; };

	ftl::rgbd::Source *host() { return host_; }

	protected:
	capability_t capabilities_;    // To be deprecated
	ftl::rgbd::Source *host_;
};

}
}

#endif  // _FTL_RGBD_DETAIL_SOURCE_HPP_
