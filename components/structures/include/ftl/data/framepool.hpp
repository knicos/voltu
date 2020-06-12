#ifndef _FTL_DATA_FRAMEPOOL_HPP_
#define _FTL_DATA_FRAMEPOOL_HPP_

#include <ftl/data/new_frame.hpp>
#include <list>
#include <unordered_map>

namespace ftl {
namespace data {

class Pool {
	public:
	explicit Pool(size_t min_n, size_t max_n);
	~Pool();

	ftl::data::Frame allocate(uint32_t id, int64_t timestamp);
	void release(Frame &f);

	ftl::data::Session &session(uint32_t id);

	size_t size(uint32_t id);

	size_t size();

	private:
	struct PoolData {
		std::list<ftl::data::Frame*> pool;
		ftl::data::Session session;
		int64_t last_timestamp;
	};

	std::unordered_map<uint32_t, PoolData> pool_;
	size_t min_n_;
	size_t max_n_;
	size_t ideal_n_;

	PoolData &_getPool(uint32_t);
};

}
}

#endif