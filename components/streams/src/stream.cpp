#include <ftl/streams/stream.hpp>

using ftl::stream::Muxer;
using ftl::stream::Broadcast;
using ftl::stream::Intercept;
using ftl::stream::Stream;

const ftl::codecs::Channels<0> &Stream::available(int fs) const {
	SHARED_LOCK(mtx_, lk);
	if (fs < 0 || fs >= state_.size()) throw ftl::exception("Frameset index out-of-bounds");
	return state_[fs].available;
}

const ftl::codecs::Channels<0> &Stream::selected(int fs) const {
	SHARED_LOCK(mtx_, lk);
	if (fs < 0 || fs >= state_.size()) throw ftl::exception("Frameset index out-of-bounds");
	return state_[fs].selected;
}

void Stream::select(int fs, const ftl::codecs::Channels<0> &s) {
	UNIQUE_LOCK(mtx_, lk);
	if (fs < 0 || fs >= state_.size()) throw ftl::exception("Frameset index out-of-bounds");
	state_[fs].selected = s;
}

ftl::codecs::Channels<0> &Stream::available(int fs) {
	UNIQUE_LOCK(mtx_, lk);
	if (fs < 0) throw ftl::exception("Frameset index out-of-bounds");
	if (fs >= state_.size()) state_.resize(fs+1);
	return state_[fs].available;
}

// ==== Muxer ==================================================================

Muxer::Muxer(nlohmann::json &config) : Stream(config), nid_(0) {

}

Muxer::~Muxer() {

}


void Muxer::add(Stream *s) {
	UNIQUE_LOCK(mutex_,lk);

	auto &se = streams_.emplace_back();
	int i = streams_.size()-1;
	se.stream = s;

	s->onPacket([this,s,i](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		//SHARED_LOCK(mutex_, lk);
		int id = _lookup(i, spkt.frame_number);
		
		ftl::codecs::StreamPacket spkt2 = spkt;
		spkt2.streamID = 0;
		spkt2.frame_number = id;
		_notify(spkt2, pkt);
		s->select(spkt.streamID, selected(0));
	});
}

bool Muxer::onPacket(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &cb) {
	UNIQUE_LOCK(mutex_,lk);
	cb_ = cb;
	return true;
}

bool Muxer::post(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	SHARED_LOCK(mutex_, lk);
	available(spkt.frameSetID()) += spkt.channel;
	
	if (spkt.frame_number < revmap_.size()) {
		auto [sid, ssid] = revmap_[spkt.frame_number];
		auto &se = streams_[sid];

		ftl::codecs::StreamPacket spkt2 = spkt;
		spkt2.streamID = 0;
		spkt2.frame_number = ssid;
		se.stream->select(0, selected(spkt.frameSetID()));
		return se.stream->post(spkt2, pkt);
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

int Muxer::_lookup(int sid, int ssid) {
	SHARED_LOCK(mutex_, lk);
	auto &se = streams_[sid];
	if (ssid >= se.maps.size()) {
		lk.unlock();
		{
			UNIQUE_LOCK(mutex_, lk2);
			int nid = nid_++;
			se.maps.push_back(nid);
			revmap_.push_back({sid,ssid});
		}
		lk.lock();
	}
	return se.maps[ssid];
}

void Muxer::_notify(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	SHARED_LOCK(mutex_, lk);
	available(spkt.frameSetID()) += spkt.channel;
	if (cb_) cb_(spkt, pkt);
}

// ==== Broadcaster ============================================================

Broadcast::Broadcast(nlohmann::json &config) : Stream(config) {

}

Broadcast::~Broadcast() {

}

void Broadcast::add(Stream *s) {
	UNIQUE_LOCK(mutex_,lk);

	streams_.push_back(s);
	s->onPacket([this](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		SHARED_LOCK(mutex_, lk);
		if (cb_) cb_(spkt, pkt);
	});
}

bool Broadcast::onPacket(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &cb) {
	UNIQUE_LOCK(mutex_,lk);
	cb_ = cb;
	return true;
}

bool Broadcast::post(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	SHARED_LOCK(mutex_, lk);
	
	bool status = true;
	for (auto s : streams_) {
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
	bool r = true;
	for (auto &s : streams_) {
		r = r && s->active();
	}
	return r;
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
	s->onPacket([this](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		SHARED_LOCK(mutex_, lk);
		available(spkt.frameSetID()) += spkt.channel;
		if (cb_) cb_(spkt, pkt);
		if (intercept_) intercept_(spkt, pkt);
		stream_->select(spkt.streamID, selected(spkt.streamID));
	});
}

bool Intercept::onPacket(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &cb) {
	UNIQUE_LOCK(mutex_,lk);
	cb_ = cb;
	return true;
}

bool Intercept::onIntercept(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &cb) {
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
