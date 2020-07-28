#ifndef _FTL_DATA_FRAMEPOOL_HPP_
#define _FTL_DATA_FRAMEPOOL_HPP_

#include <ftl/data/new_frame.hpp>
#include <ftl/data/new_frameset.hpp>
#include <ftl/data/creators.hpp>
#include <list>
#include <unordered_map>

namespace ftl {
namespace data {

class Pool {
	friend class Session;
	friend class FrameSet;

	public:
	explicit Pool(size_t min_n, size_t max_n);
	~Pool();

	ftl::data::Frame allocate(FrameID id, int64_t timestamp);
	void release(Frame &f);

	ftl::data::Session &session(FrameID id);
	inline ftl::data::Session &group(FrameID id) { return session(id); }

	inline ftl::Handle onFlush(const std::function<bool(ftl::data::Frame&,ftl::codecs::Channel)> &cb) { return flush_.on(cb); }

	inline ftl::Handle onFlushSet(const std::function<bool(ftl::data::FrameSet&,ftl::codecs::Channel)> &cb) { return flush_fs_.on(cb); }

	size_t size(FrameID id);

	size_t size();

	template <typename T, typename ...ARGS>
	T creator(FrameID id, ARGS ...args) {
		static_assert(std::is_base_of<ftl::data::FrameCreator, T>::value, "A creator must inherit FrameCreator");
		return T(this, id, args...);
	}

	private:
	struct PoolData {
		std::list<ftl::data::Frame*> pool;
		ftl::data::Session session;
		int64_t last_timestamp=0;
	};

	std::unordered_map<uint32_t, PoolData> pool_;
	size_t min_n_;
	size_t max_n_;
	size_t ideal_n_;

	ftl::Handler<ftl::data::Frame&,ftl::codecs::Channel> flush_;
	ftl::Handler<ftl::data::FrameSet&,ftl::codecs::Channel> flush_fs_;

	MUTEX mutex_;

	PoolData &_getPool(FrameID);
};

}
}

#endif