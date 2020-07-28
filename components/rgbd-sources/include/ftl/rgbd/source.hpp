#ifndef _FTL_RGBD_SOURCE_HPP_
#define _FTL_RGBD_SOURCE_HPP_

#include <ftl/cuda_util.hpp>
#include <ftl/configuration.hpp>
#include <ftl/threads.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/uri.hpp>
#include <ftl/rgbd/camera.hpp>
#include <opencv2/core/mat.hpp>
#include <Eigen/Eigen>
#include <string>
#include <map>

#include <ftl/cuda_common.hpp>
#include <ftl/data/new_frame.hpp>
#include <ftl/data/new_frameset.hpp>
#include <ftl/data/creators.hpp>

namespace ftl {

namespace net {
class Universe;
}

namespace rgbd {

class BaseSourceImpl;

static inline bool isValidDepth(float d) { return (d > 0.01f) && (d < 39.99f); }

/**
 * RGBD Generic data source configurable entity. This class hides the
 * internal implementation of an RGBD source by providing accessor functions
 * and by automatically changing the implementation in response to any URI
 * changes.
 *
 * Cannot be constructed directly, use ftl::create<Source>(...).
 * @see ftl::create
 */
class Source : public ftl::Configurable, public ftl::data::DiscreteSource {
	public:
	template <typename T, typename... ARGS>
	friend T *ftl::config::create(ftl::config::json_t &, ARGS ...);

	// This class cannot be constructed directly, use ftl::create
	Source()=delete;

	// Also cannot be copied
	Source(const Source&)=delete;
	Source &operator=(const Source&) =delete;

	protected:
	explicit Source(ftl::config::json_t &cfg);

	public:
	virtual ~Source();
	
	/**
	 * Is this source valid and ready to grab?.
	 */
	bool isReady();

	/**
	 * Perform the hardware or virtual frame grab operation. This should be
	 * fast and non-blocking.
	 */
	bool capture(int64_t ts) override;

	/**
	 * Download captured frame. This could be a blocking IO operation.
	 */
	bool retrieve(ftl::data::Frame &) override;

	/**
	 * Force the internal implementation to be reconstructed.
	 */
	void reset();

	std::string getURI() { return value("uri", std::string("")); }

	SHARED_MUTEX &mutex() { return mutex_; }

	/**
	 * Check whether a given device URI is supported. This will check hardware
	 * for physical availability of devices.
	 */
	static bool supports(const std::string &uri);


	private:
	BaseSourceImpl *impl_;
	SHARED_MUTEX mutex_;
	cudaStream_t stream_;
	std::atomic_bool is_retrieving;

	void _swap();
};

}
}

#endif  // _FTL_RGBD_SOURCE_HPP_
