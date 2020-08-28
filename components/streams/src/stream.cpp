#include <ftl/streams/stream.hpp>

#define LOGURU_WITH_STREAMS 1
#include <loguru.hpp>

using ftl::stream::Muxer;
using ftl::stream::Broadcast;
using ftl::stream::Intercept;
using ftl::stream::Stream;

std::unordered_set<ftl::codecs::Channel> operator&(const std::unordered_set<ftl::codecs::Channel> &a, const std::unordered_set<ftl::codecs::Channel> &b) {
	std::unordered_set<ftl::codecs::Channel> result;
	for (auto &i : a) {
		if (b.find(i) != b.end()) result.insert(i);
	}
	return result;
}

std::unordered_set<ftl::codecs::Channel> operator-(const std::unordered_set<ftl::codecs::Channel> &a, const std::unordered_set<ftl::codecs::Channel> &b) {
	std::unordered_set<ftl::codecs::Channel> result;
	for (auto &i : a) {
		if (b.find(i) == b.end()) result.insert(i);
	}
	return result;
}

bool operator!=(const std::unordered_set<ftl::codecs::Channel> &a, const std::unordered_set<ftl::codecs::Channel> &b) {
	if (a.size() != b.size()) return true;
	for (auto &i : a) {
		if (b.count(i) == 0) return true;
	}
	return false;
}

const std::unordered_set<ftl::codecs::Channel> &Stream::available(int fs) const {
	SHARED_LOCK(mtx_, lk);
	if (fs < 0 || static_cast<uint32_t>(fs) >= state_.size()) throw FTL_Error("Frameset index out-of-bounds: " << fs);
	return state_[fs].available;
}

const std::unordered_set<ftl::codecs::Channel> &Stream::selected(int fs) const {
	SHARED_LOCK(mtx_, lk);
	if (fs < 0 || static_cast<uint32_t>(fs) >= state_.size()) throw FTL_Error("Frameset index out-of-bounds: " << fs);
	return state_[fs].selected;
}

std::unordered_set<ftl::codecs::Channel> Stream::selectedNoExcept(int fs) const {
	if (fs == 255) return {};

	SHARED_LOCK(mtx_, lk);
	if (fs < 0 || static_cast<uint32_t>(fs) >= state_.size()) return {};
	return state_[fs].selected;
}

void Stream::select(int fs, const std::unordered_set<ftl::codecs::Channel> &s, bool make) {
	if (fs == 255) return;

	UNIQUE_LOCK(mtx_, lk);
	if (fs < 0 || (!make && static_cast<uint32_t>(fs) >= state_.size())) throw FTL_Error("Frameset index out-of-bounds: " << fs);
	if (static_cast<uint32_t>(fs) >= state_.size()) state_.resize(fs+1);
	state_[fs].selected = s;
}

std::unordered_set<ftl::codecs::Channel> &Stream::available(int fs) {
	UNIQUE_LOCK(mtx_, lk);
	if (fs < 0) throw FTL_Error("Frameset index out-of-bounds: " << fs);
	if (static_cast<uint32_t>(fs) >= state_.size()) state_.resize(fs+1);
	return state_[fs].available;
}

void Stream::reset() {
	// Clear available and selected?
}

// ==== Muxer ==================================================================

Muxer::Muxer(nlohmann::json &config) : Stream(config), nid_{0} {

}

Muxer::~Muxer() {
	UNIQUE_LOCK(mutex_,lk);
	for (auto &se : streams_) {
		se.handle.cancel();
	}
}


void Muxer::add(Stream *s, size_t fsid, const std::function<int()> &cb) {
	UNIQUE_LOCK(mutex_,lk);
	if (fsid < 0u || fsid >= ftl::stream::kMaxStreams) return;

	auto &se = streams_.emplace_back();
	//int i = streams_.size()-1;
	se.stream = s;
	se.ids.push_back(fsid);
	ftl::stream::Muxer::StreamEntry *ptr = &se;

	se.handle = std::move(s->onPacket([this,s,ptr,cb](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		//TODO: Allow input streams to have other streamIDs
		// Same fsid means same streamIDs map together in the end

		/*ftl::stream::Muxer::StreamEntry *ptr = nullptr;
		{
			SHARED_LOCK(mutex_,lk);
			ptr = &streams_[i];
		}*/

		if (!cb && spkt.streamID > 0) {
			LOG(WARNING) << "Multiple framesets in stream";
			return true;
		}

		if (ptr->ids.size() <= spkt.streamID) {
			UNIQUE_LOCK(mutex_,lk);
			if (ptr->ids.size() <= spkt.streamID) {
				ptr->ids.resize(spkt.streamID + 1);
				ptr->ids[spkt.streamID] = cb();
			}
		}

		int fsid;
		{
			SHARED_LOCK(mutex_, lk);
			fsid = ptr->ids[spkt.streamID];
		}

		ftl::codecs::StreamPacket spkt2 = spkt;
		ptr->original_fsid = spkt.streamID;  // FIXME: Multiple originals needed
		spkt2.streamID = fsid;

		if (spkt2.frame_number < 255) {
			int id = _lookup(fsid, ptr, spkt.frame_number, pkt.frame_count);
			spkt2.frame_number = id;
		}

		_notify(spkt2, pkt);
		s->select(spkt.streamID, selected(fsid), true);
		return true;
	}));
}

void Muxer::remove(Stream *s) {
	UNIQUE_LOCK(mutex_,lk);
	for (auto i = streams_.begin(); i != streams_.end(); ++i) {
		if (i->stream == s) {
			i->handle.cancel();
			auto *se = &(*i);

			for (size_t j=0; j<kMaxStreams; ++j) {
				for (auto &k : revmap_[j]) {
					if (k.first == se) {
						k.first = nullptr;
					}
				}
			}

			streams_.erase(i);
			return;
		}
	}
}

ftl::stream::Stream *Muxer::originStream(size_t fsid, int fid) {
	if (fsid < ftl::stream::kMaxStreams && static_cast<uint32_t>(fid) < revmap_[fsid].size()) {
		return std::get<0>(revmap_[fsid][fid])->stream;
	}
	return nullptr;
}

bool Muxer::post(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	SHARED_LOCK(mutex_, lk);
	if (pkt.data.size() > 0 || !(spkt.flags & ftl::codecs::kFlagRequest)) available(spkt.frameSetID()) += spkt.channel;

	if (spkt.streamID < ftl::stream::kMaxStreams && spkt.frame_number < revmap_[spkt.streamID].size()) {
		auto [se, ssid] = revmap_[spkt.streamID][spkt.frame_number];
		//auto &se = streams_[sid];

		if (!se) return false;

		//LOG(INFO) << "POST " << spkt.frame_number;

		ftl::codecs::StreamPacket spkt2 = spkt;
		spkt2.streamID = se->original_fsid;  // FIXME: Multiple possible originals
		spkt2.frame_number = ssid;
		se->stream->select(spkt2.streamID, selected(spkt.frameSetID()));
		return se->stream->post(spkt2, pkt);
	} else {
		return false;
	}
}

bool Muxer::begin() {
	bool r = true;
	for (auto &s : streams_) {
		r = r && s.stream->begin();
	}
	return r;
}

bool Muxer::end() {
	bool r = true;
	for (auto &s : streams_) {
		r = r && s.stream->end();
	}
	return r;
}

bool Muxer::active() {
	bool r = true;
	for (auto &s : streams_) {
		r = r && s.stream->active();
	}
	return r;
}

void Muxer::reset() {
	for (auto &s : streams_) {
		s.stream->reset();
	}
}

int Muxer::_lookup(size_t fsid, ftl::stream::Muxer::StreamEntry *se, int ssid, int count) {
	SHARED_LOCK(mutex_, lk);
	
	auto i = se->maps.find(fsid);
	if (i == se->maps.end()) {
		lk.unlock();
		{
			UNIQUE_LOCK(mutex_, lk2);
			if (se->maps.count(fsid) == 0) {
				se->maps[fsid] = {};
			}
			i = se->maps.find(fsid);
		}
		lk.lock();
	}

	auto &map = i->second;

	if (static_cast<uint32_t>(ssid) >= map.size()) {
		lk.unlock();
		{
			UNIQUE_LOCK(mutex_, lk2);
			while (static_cast<uint32_t>(ssid) >= map.size()) {
				int nid = nid_[fsid]++;
				revmap_[fsid].push_back({se, static_cast<uint32_t>(map.size())});
				map.push_back(nid);
				for (int i=1; i<count; ++i) {
					int nid = nid_[fsid]++;
					revmap_[fsid].push_back({se, static_cast<uint32_t>(map.size())});
					map.push_back(nid);
				}
			}
		}
		lk.lock();
	}
	return map[ssid];
}

void Muxer::_notify(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	SHARED_LOCK(mutex_, lk);
	available(spkt.frameSetID()) += spkt.channel;

	try {
		cb_.trigger(spkt, pkt);  // spkt.frame_number < 255 &&
	} catch (std::exception &e) {
		LOG(ERROR) << "Exception in packet handler (" << int(spkt.channel) << "): " << e.what();
		//reset();  // Force stream reset here to get new i-frames
	}
}

// ==== Broadcaster ============================================================

Broadcast::Broadcast(nlohmann::json &config) : Stream(config) {

}

Broadcast::~Broadcast() {

}

void Broadcast::add(Stream *s) {
	UNIQUE_LOCK(mutex_,lk);

	streams_.push_back(s);
	handles_.push_back(std::move(s->onPacket([this,s](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		//LOG(INFO) << "BCAST Request: " << (int)spkt.streamID << " " << (int)spkt.channel << " " << spkt.timestamp;
		SHARED_LOCK(mutex_, lk);
		if (spkt.frameSetID() < 255) available(spkt.frameSetID()) += spkt.channel;
		cb_.trigger(spkt, pkt);
		if (spkt.streamID < 255) s->select(spkt.streamID, selected(spkt.streamID));
		return true;
	})));
}

void Broadcast::remove(Stream *s) {
	UNIQUE_LOCK(mutex_,lk);
	// TODO: Find and remove handle also
	streams_.remove(s);
}

void Broadcast::clear() {
	UNIQUE_LOCK(mutex_,lk);
	handles_.clear();
	streams_.clear();
}

bool Broadcast::post(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	SHARED_LOCK(mutex_, lk);
	if (spkt.frameSetID() < 255) available(spkt.frameSetID()) += spkt.channel;

	bool status = true;
	for (auto s : streams_) {
		//s->select(spkt.frameSetID(), selected(spkt.frameSetID()));
		status = status && s->post(spkt, pkt);
	}
	return status;
}

bool Broadcast::begin() {
	bool r = true;
	for (auto &s : streams_) {
		r = r && s->begin();
	}
	return r;
}

bool Broadcast::end() {
	bool r = true;
	for (auto &s : streams_) {
		r = r && s->end();
	}
	return r;
}

bool Broadcast::active() {
	if (streams_.size() == 0) return false;
	bool r = true;
	for (auto &s : streams_) {
		r = r && s->active();
	}
	return r;
}

void Broadcast::reset() {
	for (auto &s : streams_) {
		s->reset();
	}
}

// ==== Intercept ==============================================================

Intercept::Intercept(nlohmann::json &config) : Stream(config) {
	stream_ = nullptr;
}

Intercept::~Intercept() {

}

void Intercept::setStream(Stream *s) {
	UNIQUE_LOCK(mutex_,lk);

	stream_ = s;
	handles_.push_back(std::move(s->onPacket([this](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		SHARED_LOCK(mutex_, lk);
		available(spkt.frameSetID()) += spkt.channel;
		cb_.trigger(spkt, pkt);
		if (intercept_) intercept_(spkt, pkt);
		stream_->select(spkt.streamID, selected(spkt.streamID));
		return true;
	})));
}

bool Intercept::onIntercept(const std::function<bool(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &cb) {
	UNIQUE_LOCK(mutex_,lk);
	intercept_ = cb;
	return true;
}

bool Intercept::post(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	SHARED_LOCK(mutex_, lk);
	available(spkt.frameSetID()) += spkt.channel;
	stream_->select(spkt.streamID, selected(spkt.streamID));
	//if (intercept_) intercept_(spkt, pkt);
	return stream_->post(spkt, pkt);
}

bool Intercept::begin() {
	return stream_->begin();
}

bool Intercept::end() {
	return stream_->end();
}

bool Intercept::active() {
	return stream_->active();
}

void Intercept::reset() {
	stream_->reset();
}
