#include <ftl/data/framepool.hpp>

using ftl::data::Pool;
using ftl::data::Frame;
using ftl::data::Session;

Pool::Pool(size_t min_n, size_t max_n) : min_n_(min_n), max_n_(max_n) {
	ideal_n_ = min_n + (max_n-min_n)/2;
}

Pool::~Pool() {
	UNIQUE_LOCK(mutex_, lk);
	for (auto &p : pool_) {
		for (auto *f : p.second.pool) {
			f->status_ = FrameStatus::RELEASED;
			delete f;
		}
	}
}

Frame Pool::allocate(FrameID id, int64_t timestamp) {
	Frame *f;

	{
		UNIQUE_LOCK(mutex_, lk);
		auto &pool = _getPool(id);

		if (timestamp < pool.last_timestamp) {
			timestamp = pool.last_timestamp;
			//throw FTL_Error("New frame timestamp is older than previous: " << timestamp << " vs " << pool.last_timestamp);
		}

		// Add items as required
		if (pool.pool.size() < min_n_) {
			while (pool.pool.size() < ideal_n_) {
				pool.pool.push_back(new Frame(this, &pool.session, id, 0));
			}
		}

		f = pool.pool.front();
		pool.pool.pop_front();
		pool.last_timestamp = timestamp;
	}

	Frame ff = std::move(*f);
	ff.restart(timestamp);
	delete f;

	return ff;
}

void Pool::release(Frame &f) {
	if (f.status() == FrameStatus::RELEASED) return;
	f.reset();

	UNIQUE_LOCK(mutex_, lk);
	auto &pool = _getPool(f.id());

	if (pool.pool.size() < max_n_) {
		Frame *pf = new Frame(this, &pool.session, f.id(), 0);
		f.moveTo(*pf);
		pool.pool.push_back(pf);
	}
}

ftl::data::Session &Pool::session(FrameID id) {
	UNIQUE_LOCK(mutex_, lk);
	auto &pool = _getPool(id);
	return pool.session;
}

size_t Pool::size(FrameID id) {
	UNIQUE_LOCK(mutex_, lk);
	auto &pool = _getPool(id);
	return pool.pool.size();
}

size_t Pool::size() {
	UNIQUE_LOCK(mutex_, lk);
	size_t s = 0;
	for (auto &p : pool_) {
		s += p.second.pool.size();
	}
	return s;
}

ftl::data::Pool::PoolData &Pool::_getPool(FrameID id) {
	return pool_[id.id];
}
