#ifndef _FTL_DATA_FRAMECREATOR_HPP_
#define _FTL_DATA_FRAMECREATOR_HPP_

#include <ftl/handle.hpp>
#include <ftl/data/new_frame.hpp>

namespace ftl {
namespace data {

class Pool;

/**
 * Create frames on demand.
 */
class FrameCreator {
	friend class Pool;

	public:
	Frame create();
	Frame create(int64_t timestamp);

	inline uint32_t id() const { return id_; }
	inline Pool *pool() const { return pool_; }

	protected:
	FrameCreator(Pool *p_pool, FrameID p_id) : pool_(p_pool), id_(p_id) {}

	private:
	Pool *pool_;
	FrameID id_;
};

/**
 * Abstract class for discrete data sources involving a high precision capture
 * and slower retrieve step. This works for both cameras and audio sources.
 */
class DiscreteSource {
	public:
	virtual bool capture(int64_t ts)=0;
	virtual bool retrieve(ftl::data::Frame &)=0;
};

/**
 * Create frames at the global frame rate with both capture and retrieve steps.
 * A source should implement this
 */
class IntervalFrameCreator : public ftl::data::FrameCreator {
	friend class Pool;

	private:
	explicit IntervalFrameCreator(Pool *p_pool, FrameID p_id, DiscreteSource *src);

	public:

	void start();
	void stop();

	private:
	ftl::Handle capture_;
	ftl::Handle retrieve_;
	DiscreteSource *src_;
};

}
}

#endif