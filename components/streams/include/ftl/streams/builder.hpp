#ifndef _FTL_STREAM_BUILDER_HPP_
#define _FTL_STREAM_BUILDER_HPP_

#include <ftl/data/new_frameset.hpp>
#include <ftl/data/framepool.hpp>
#include <ftl/handle.hpp>
#include <ftl/transactional.hpp>
#include <list>

namespace ftl {
namespace streams {

using LockedFrameSet = ftl::Transactional<ftl::data::FrameSet*>;

/**
 * An abstract base class for a FrameSet database. A builder stores incomplete
 * framesets whilst they are being created, allowing partial data to be buffered
 * and searched for using timestamp and frameset id. One instance of a builder
 * should be created for each frameset id.
 */
class BaseBuilder : public ftl::data::Generator {
	public:
	BaseBuilder(ftl::data::Pool *pool, int id);
	BaseBuilder();
	virtual ~BaseBuilder();

	virtual LockedFrameSet get(int64_t timestamp, size_t ix)=0;

	virtual LockedFrameSet get(int64_t timestamp)=0;

	//void setName(const std::string &name);

	void setID(uint32_t id) { id_ = id; }
	void setPool(ftl::data::Pool *p) { pool_ = p; }
	void setBufferSize(size_t s) { bufferSize_ = s; }

	inline ftl::Handle onFrameSet(const ftl::data::FrameSetCallback &cb) override { return cb_.on(cb); }

	/**
	 * Retrieve an fps + latency pair, averaged since last call to this
	 * function.
	 */
	static std::pair<float,float> getStatistics();

	inline size_t size() const { return size_; }

	inline const int id() const { return id_; }

	inline const ftl::data::ChangeType changeType() const { return ctype_; }

	protected:
	ftl::data::Pool *pool_;
	int id_;
	size_t size_;
	size_t bufferSize_ = 1;
	ftl::Handler<const ftl::data::FrameSetPtr&> cb_;
	ftl::data::ChangeType ctype_ = ftl::data::ChangeType::COMPLETED;
};

/**
 * A version of the frameset database that is used by sources or renderers to
 * obtain new frames. Timestamps are not used in this version as only a single
 * frameset is being buffered.
 */
class LocalBuilder : public BaseBuilder {
	public:
	LocalBuilder(ftl::data::Pool *pool, int id);
	LocalBuilder();
	virtual ~LocalBuilder();

	LockedFrameSet get(int64_t timestamp, size_t ix) override;

	LockedFrameSet get(int64_t timestamp) override;

	void setFrameCount(size_t size);

	/**
	 * Return a smart pointer to a new frameset. The frameset will have the
	 * number of frames set with `setFrameCount`, or 1 frame by default. Once
	 * called, another new frameset is buffered internally and ownership of the
	 * returned frameset is transfered.
	 */
	std::shared_ptr<ftl::data::FrameSet> getNextFrameSet(int64_t ts);

	private:
	std::shared_ptr<ftl::data::FrameSet> frameset_;
	SHARED_MUTEX mtx_;

	std::shared_ptr<ftl::data::FrameSet> _allocate(int64_t timestamp);
};

/**
 * A local builder that generates framesets using a timer and populates the
 * frames using a discrete source object before generating a callback.
 */
class IntervalSourceBuilder : public LocalBuilder {
	public:
	IntervalSourceBuilder(ftl::data::Pool *pool, int id, ftl::data::DiscreteSource *src);
	IntervalSourceBuilder(ftl::data::Pool *pool, int id, const std::list<ftl::data::DiscreteSource*> &srcs);
	IntervalSourceBuilder();
	~IntervalSourceBuilder();

	void start();
	void stop();

	private:
	ftl::Handle capture_;
	ftl::Handle retrieve_;
	std::list<ftl::data::DiscreteSource *> srcs_;
};

/**
 * A local builder that generates framesets manually and populates the
 * frames using a discrete source object before generating a callback.
 */
class ManualSourceBuilder : public LocalBuilder {
	public:
	ManualSourceBuilder(ftl::data::Pool *pool, int id, ftl::data::DiscreteSource *src);
	ManualSourceBuilder();
	~ManualSourceBuilder();

	void tick();

	inline void setFrameRate(int fps) { mspf_ = 1000/fps; };

	private:
	ftl::data::DiscreteSource *src_;
	int mspf_ = 30;
	int64_t last_timestamp_=0;
};

class ForeignBuilder : public BaseBuilder {
	public:
	ForeignBuilder(ftl::data::Pool *pool, int id);
	ForeignBuilder();
	~ForeignBuilder();

	//inline void setID(int id) { id_ = id; }

	LockedFrameSet get(int64_t timestamp, size_t ix) override;

	LockedFrameSet get(int64_t timestamp) override;

	private:
	std::list<std::shared_ptr<ftl::data::FrameSet>> framesets_;  // Active framesets
	//std::list<ftl::data::FrameSet*> allocated_;  // Keep memory allocations

	size_t head_;
	MUTEX mutex_;
	int mspf_;
	int64_t last_ts_;
	int64_t last_frame_;
	std::atomic<int> jobs_;
	volatile bool skip_;
	ftl::Handle main_id_;

	std::string name_;

	static MUTEX msg_mutex__;
	static float latency__;
	static float fps__;
	static int stats_count__;

	/* Insert a new frameset into the buffer, along with all intermediate
	 * framesets between the last in buffer and the new one.
	 */
	std::shared_ptr<ftl::data::FrameSet> _addFrameset(int64_t timestamp);

	/* Find a frameset with given latency in frames. */
	std::shared_ptr<ftl::data::FrameSet> _getFrameset();
	std::shared_ptr<ftl::data::FrameSet> _get(int64_t timestamp);

	/* Search for a matching frameset. */
	std::shared_ptr<ftl::data::FrameSet> _findFrameset(int64_t ts);
	//void _freeFrameset(std::shared_ptr<ftl::data::FrameSet>);

	void _schedule();

	//void _recordStats(float fps, float latency);
};

}
}

#endif
