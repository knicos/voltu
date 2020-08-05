#include <ftl/data/new_frame.hpp>
#include <ftl/data/framepool.hpp>
#include <ftl/timer.hpp>

using ftl::data::Frame;
using ftl::data::Session;
using ftl::data::ChannelConfig;
using ftl::data::StorageMode;
using ftl::data::FrameStatus;
using ftl::codecs::Channel;
using ftl::data::Message;

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

static std::unordered_map<ftl::codecs::Channel, ChannelConfig> reg_channels;
static std::unordered_map<size_t, std::function<bool(const ftl::data::Frame &, ftl::codecs::Channel, std::vector<uint8_t> &)>> encoders;

void ftl::data::registerChannel(ftl::codecs::Channel c, const ChannelConfig &config) {
	auto i = reg_channels.find(c);
	if (i != reg_channels.end()) {
		if (i->second.mode == config.mode && i->second.type_id == config.type_id && i->second.name == config.name) {
			return;
		}
		throw FTL_Error("Channel " << static_cast<unsigned int>(c) << " already registered");
	}

	reg_channels[c] = config;
}

void ftl::data::clearRegistry() {
	reg_channels.clear();
}

bool ftl::data::isPersistent(ftl::codecs::Channel c) {
	auto i = reg_channels.find(c);
	return (i != reg_channels.end()) ? i->second.mode == StorageMode::PERSISTENT : int(c) >= 64 && int(c) < 2048;
}

bool ftl::data::isAggregate(ftl::codecs::Channel c) {
	auto i = reg_channels.find(c);
	return (i != reg_channels.end()) ? i->second.mode == StorageMode::AGGREGATE : (int(c) >= 32 && int(c) < 64) || int(c) >= 4096;
}

size_t ftl::data::getChannelType(ftl::codecs::Channel c) {
	auto i = reg_channels.find(c);
	return (i != reg_channels.end()) ? i->second.type_id : 0;
}

std::string ftl::data::getChannelName(ftl::codecs::Channel c) {
	auto i = reg_channels.find(c);
	return (i != reg_channels.end()) ? i->second.name : "";
}

ftl::codecs::Channel ftl::data::getChannelByName(const std::string &name) {
	return ftl::codecs::Channel::Colour;
}

std::function<bool(const ftl::data::Frame &, ftl::codecs::Channel, std::vector<uint8_t> &)> ftl::data::getTypeEncoder(size_t type) {
	const auto &i = encoders.find(type);
	if (i != encoders.end()) return i->second;
	else return nullptr;
}

void ftl::data::setTypeEncoder(size_t type, const std::function<bool(const ftl::data::Frame &, ftl::codecs::Channel, std::vector<uint8_t> &)> &e) {
	encoders[type] = e;
}

//==============================================================================

//static std::atomic_int frame_count = 0;

Frame::Frame(Pool *ppool, Session *parent, FrameID pid, int64_t ts)
 : timestamp_(ts), id_(pid), pool_(ppool), parent_(parent), status_(FrameStatus::CREATED) {
	//LOG(INFO) << "Frames: " << ++frame_count;
 };

Frame::~Frame() {
	if (status_ == FrameStatus::CREATED) store();
	if (status_ == FrameStatus::STORED) flush();
	if (status_ != FrameStatus::RELEASED && pool_) {
		pool_->release(*this);
		//--frame_count;
	}
};

bool ftl::data::Frame::hasAll(const std::unordered_set<ftl::codecs::Channel> &cs) {
	for (auto &a : cs) {
		if (!has(a)) return false;
	}
	return true;
}

bool ftl::data::Frame::has(ftl::codecs::Channel c) const {
	const auto &i = data_.find(c);
	if (i != data_.end() && i->second.status != ftl::data::ChannelStatus::INVALID) {
		return true;
	} else {
		return (parent_ && parent_->has(c));
	}
}

bool ftl::data::Frame::availableAll(const std::unordered_set<ftl::codecs::Channel> &cs) const {
	bool result = true;
	for (auto c : cs) {
		result &= available(c);
	}
	return result;
}

std::unordered_set<ftl::codecs::Channel> ftl::data::Frame::available() const {
	std::unordered_set<ftl::codecs::Channel> result = channels();

	uint64_t m = 1;
	// TODO: NAIVE, use ffs or ctz.
	for (int i=0; i<32; ++i) {
		if (m & available_) result.emplace(static_cast<Channel>(i));
		m <<= 1;
	}

	return result;
}

void ftl::data::Frame::remove(ftl::codecs::Channel c) {
	const auto &i = data_.find(c);
	if (i != data_.end()) {
		i->second.status = ftl::data::ChannelStatus::INVALID;
		changed_.erase(c);
	}
}

Frame::ChannelData &Frame::_getData(ftl::codecs::Channel c) {
	if (status_ == FrameStatus::RELEASED) throw FTL_Error("Reading a released frame");
	const auto &i = data_.find(c);
	if (i != data_.end() && i->second.status != ChannelStatus::INVALID) {
		return i->second;
	} else if (parent_) {
		return parent_->_getData(c);
	} else throw FTL_Error("Missing channel (" << static_cast<unsigned int>(c) << ")");
}

const Frame::ChannelData &Frame::_getData(ftl::codecs::Channel c) const {
	if (status_ == FrameStatus::RELEASED) throw FTL_Error("Reading a released frame");
	const auto &i = data_.find(c);
	if (i != data_.end() && i->second.status != ChannelStatus::INVALID) {
		return i->second;
	} else if (parent_) {
		return parent_->_getData(c);
	} else throw FTL_Error("Missing channel (" << static_cast<unsigned int>(c) << ")");
}

std::any &Frame::createAnyChange(ftl::codecs::Channel c, ftl::data::ChangeType t) {
	if (status_ != FrameStatus::CREATED) throw FTL_Error("Cannot apply change after store " << static_cast<int>(status_));

	ftl::data::Frame::ChannelData *d;

	if (parent_) {
		UNIQUE_LOCK(mutex(), lk);
		d = &(data_[c]);
		touch(c, t);
	} else {
		d = &(data_[c]);
		touch(c, t);
	}

	if (d->status != ftl::data::ChannelStatus::FLUSHED) {
		d->status = ftl::data::ChannelStatus::DISPATCHED;
		d->encoded.clear();
		return d->data;
	} else {
		throw FTL_Error("Channel is flushed and read-only: " << static_cast<unsigned int>(c));
	}
}

std::any &Frame::createAnyChange(ftl::codecs::Channel c, ftl::data::ChangeType t, const ftl::codecs::Packet &data) {
	if (status_ != FrameStatus::CREATED) throw FTL_Error("Cannot apply change after store " << static_cast<int>(status_));

	ftl::data::Frame::ChannelData *d;

	if (parent_) {
		UNIQUE_LOCK(mutex(), lk);
		d = &(data_[c]);
		touch(c, t);
	} else {
		d = &(data_[c]);
		touch(c, t);
	}

	if (d->status != ftl::data::ChannelStatus::FLUSHED) {
		d->status = (data.codec == ftl::codecs::codec_t::MSGPACK) ? ftl::data::ChannelStatus::ENCODED : ftl::data::ChannelStatus::DISPATCHED;
		d->encoded.push_back(data);
		return d->data;
	} else {
		throw FTL_Error("Channel is flushed and read-only: " << static_cast<unsigned int>(c));
	}
}

std::any &Frame::createAny(ftl::codecs::Channel c) {
	if (status_ != FrameStatus::STORED) throw FTL_Error("Cannot create before store or after flush");

	ftl::data::Frame::ChannelData *d;

	if (parent_) {
		UNIQUE_LOCK(mutex(), lk);
		d = &(data_[c]);
		touch(c);
	} else {
		d = &(data_[c]);
		touch(c);
	}

	if (d->status != ftl::data::ChannelStatus::FLUSHED) {
		d->status = ftl::data::ChannelStatus::VALID;
		d->encoded.clear();
		return d->data;
	} else {
		throw FTL_Error("Channel is flushed and read-only: " << static_cast<unsigned int>(c));
	}
}

std::any &Frame::getAnyMutable(ftl::codecs::Channel c) {
	auto &d = _getData(c);
	return d.data;
}

const std::any &Frame::getAny(ftl::codecs::Channel c) const {
	auto &d = _getData(c);
	return d.data;
}

const std::list<ftl::codecs::Packet> &ftl::data::Frame::getEncoded(ftl::codecs::Channel c) const {
	const auto &d = _getData(c);
	if (d.status != ftl::data::ChannelStatus::INVALID) {
		return d.encoded;
	} else throw FTL_Error("Missing channel (" << static_cast<unsigned int>(c) << ")");
}

bool Frame::flush() {
	if (status_ == FrameStatus::CREATED) throw FTL_Error("Frame cannot be flushed before store");
	if (status_ == FrameStatus::FLUSHED) throw FTL_Error("Frame cannot be flushed twice");
	status_ = FrameStatus::FLUSHED;

	if (parent_) {
		parent_->flush(*this);
	}
	for (auto c : changed_) {
		_getData(c.first).status = ChannelStatus::FLUSHED;
	}
	_primaryStore();
	return true;
}

bool Frame::flush(ftl::codecs::Channel c) {
	if (status_ == FrameStatus::CREATED) throw FTL_Error("Frame cannot be flushed before store");
	if (status_ == FrameStatus::FLUSHED) throw FTL_Error("Frame cannot be flushed twice");
	//status_ = FrameStatus::FLUSHED;

	if (parent_ && changed(c)) {
		parent_->flush(*this, c);
		_getData(c).status = ChannelStatus::FLUSHED;
	}
	return true;
}

void Frame::store() {
	if (status_ != FrameStatus::CREATED) throw FTL_Error("Frame cannot be stored twice");
	status_ = FrameStatus::STORED;

	if (!parent_) return;

	{
		UNIQUE_LOCK(parent_->mutex(), lk);
		for (auto c : changed_) {
			if (ftl::data::isPersistent(c.first) && hasOwn(c.first)) {
				auto &d = data_[c.first];
				auto &pd = parent_->data_[c.first];
				pd.data = std::move(d.data);
				pd.encoded = std::move(d.encoded);
				//if (d.status == ChannelStatus::ENCODED) LOG(INFO) << "STORE ENCODED: " << (int)c.first;
				pd.status = ChannelStatus::VALID;
				//data_.erase(c.first);
				d.status = ChannelStatus::INVALID;
			}
		}
	}

	for (auto c : changed_) {
		parent_->change_.trigger(*this, c.first);
		uint64_t sig = (uint64_t(id()) << 32) + static_cast<unsigned int>(c.first);
		const auto &i = parent_->change_channel_.find(sig);
		if (i != parent_->change_channel_.end()) i->second.trigger(*this, c.first);
	}
}

void Frame::_primaryStore() {
	if (mode_ == FrameMode::RESPONSE) return;
	forceStore();
}

void Frame::forceStore() {
	if (!parent_) return;

	//UNIQUE_LOCK(parent_->mutex(), lk);

	for (auto c : changed_) {
		if (ftl::data::isPersistent(c.first) && hasOwn(c.first)) {
			auto &d = data_[c.first];
			auto &pd = parent_->data_[c.first];
			pd.data = d.data;
			//pd.encoded = std::move(d.encoded);
			pd.status = ChannelStatus::VALID;
			//data_.erase(c.first);
			d.status = ChannelStatus::INVALID;
		}

		//parent_->change_.trigger(*this, c.first);
		//uint64_t sig = (uint64_t(id()) << 32) + static_cast<unsigned int>(c.first);
		//const auto &i = parent_->change_channel_.find(sig);

		//if (i != parent_->change_channel_.end()) i->second.trigger(*this, c.first);
	}
}

void Frame::merge(Frame &f) {
	for (auto &x : f) {
		auto &d = data_[x.first];
		d.data = std::move(x.second.data);
		d.encoded = std::move(x.second.encoded);
		f.data_[x.first].status = ChannelStatus::INVALID;
		d.status = ChannelStatus::VALID;
		touch(x.first);
	}
	f.status_ = FrameStatus::RELEASED;
	f.changed_.clear();
}

void Frame::moveTo(Frame &f) {
	if (status_ == FrameStatus::RELEASED) throw FTL_Error("Moving released frame");
	f.id_ = id_;
	f.timestamp_ = timestamp_;
	f.status_ = status_;
	f.mode_ = mode_;
	f.parent_ = parent_;
	f.pool_ = pool_;
	f.data_ = std::move(data_);
	f.changed_ = std::move(changed_);
	f.packet_rx = (int)packet_rx;
	f.packet_tx = (int)packet_tx;
	status_ = FrameStatus::RELEASED;
}

void Frame::swapChanged(Frame &f) {
	for (auto x : changed_) {
		f.data_[x.first].data.swap(data_[x.first].data);
		f.changed_[x.first] = (mode_ == FrameMode::PRIMARY) ? ChangeType::PRIMARY : ChangeType::RESPONSE;
	}
}

void Frame::swapChannel(ftl::codecs::Channel c, Frame &f) {
	if (f.hasOwn(c)) {
		auto &d = data_[c];
		auto &fd = f.data_[c];
		fd.data.swap(d.data);
		d.status = ftl::data::ChannelStatus::VALID;
		changed_[c] = f.changed_[c];
		f.changed_[c] = (mode_ == FrameMode::PRIMARY) ? ChangeType::PRIMARY : ChangeType::RESPONSE;
	}
}

void Frame::swapChannels(ftl::codecs::Channel c1, ftl::codecs::Channel c2) {
	if (hasOwn(c1) && hasOwn(c2)) {
		auto &d1 = data_[c1];
		auto &d2 = data_[c2];
		d2.data.swap(d1.data);

		auto status = d1.status;
		d1.status = d2.status;
		d2.status = status;

		changed_[c1] = (mode_ == FrameMode::PRIMARY) ? ChangeType::PRIMARY : ChangeType::RESPONSE;
		changed_[c2] = (mode_ == FrameMode::PRIMARY) ? ChangeType::PRIMARY : ChangeType::RESPONSE;
	}
}

void Frame::reset() {
	for (auto &d : data_) {
		d.second.status = ChannelStatus::INVALID;
		d.second.encoded.clear();

		// Note: Data channels should be cleared
		if ((int)d.first >= 32) d.second.data.reset();
	}
	changed_.clear();
	status_ = FrameStatus::CREATED;
	mode_ = FrameMode::PRIMARY;
	available_ = 0;
	packet_rx = 0;
	packet_tx = 0;
}

void Frame::hardReset() {
	status_ = FrameStatus::CREATED;
	changed_.clear();
	data_.clear();
	available_ = 0;
}

Frame Frame::response() const {
	if (!pool_) throw FTL_Error("Frame has no pool, cannot generate response");
	Frame f = pool_->allocate(id_, ftl::timer::get_time());
	f.mode_ = FrameMode::RESPONSE;
	f.store();
	return f;
}

Frame Frame::make_standalone() {
	Frame f(nullptr, nullptr, FrameID(0,0), 0);
	f.mode_ = FrameMode::STANDALONE;
	return f;
}

std::unordered_set<ftl::codecs::Channel> Frame::channels() const {
	std::unordered_set<ftl::codecs::Channel> res{};
	for (const auto& [k, v] : data_) {
		std::ignore = v;
		res.emplace(k);
	}
	return res;
}

std::unordered_set<ftl::codecs::Channel> Frame::allChannels() const {
	std::unordered_set<ftl::codecs::Channel> res{};
	for (const auto& [k, v] : data_) {
		std::ignore = v;
		res.emplace(k);
	}
	if (parent_) {
		for (const auto& [k, v] : parent_->data_) {
			std::ignore = v;
			res.emplace(k);
		}
	}

	uint64_t m = 1;
	// TODO: NAIVE, use ffs or ctz.
	for (int i=0; i<32; ++i) {
		if (m & available_) res.emplace(static_cast<Channel>(i));
		m <<= 1;
	}
	return res;
}

const std::map<ftl::data::Message,std::string> &Frame::messages() const {
	return get<std::map<ftl::data::Message,std::string>>(Channel::Messages);
}

void Frame::message(ftl::data::Message code, const std::string &msg) {
	auto &msgs = create<std::map<ftl::data::Message,std::string>>(Channel::Messages);
	msgs[code] = msg;
}

void Frame::message(ftl::data::Message code, const ftl::Formatter &msg) {
	message(code, msg.str());
}

std::string Frame::name() const {
	if (has(Channel::MetaData)) {
		const auto &meta = get<std::map<std::string,std::string>>(Channel::MetaData);
		auto i = meta.find("name");
		if (i != meta.end()) return i->second;
	}

	// Generate a name
	return std::string("Frame-") + std::to_string(frameset()) + std::string("-") + std::to_string(source());
}

const std::map<std::string,std::string> &Frame::metadata() const {
	return get<std::map<std::string,std::string>>(Channel::MetaData);
}

// ==== Session ================================================================

ftl::Handle Session::onChange(uint32_t pid, ftl::codecs::Channel c, const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
	uint64_t sig = (uint64_t(pid) << 32) + static_cast<unsigned int>(c);
	return change_channel_[sig].on(cb);
}

ftl::Handle Session::onChange(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
	return change_.on(cb);
}

ftl::Handle Session::onFlush(const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
	return flush_.on(cb);
}

void Session::notifyChanges(Frame &f) {

}

void Session::flush(Frame &f) {
	for (auto c : f.changed()) {
		if (c.second == ftl::data::ChangeType::PRIMARY || c.second == ftl::data::ChangeType::RESPONSE) {
			auto &d = f._getData(c.first);
			if (d.status == ftl::data::ChannelStatus::VALID) {
				d.status = ftl::data::ChannelStatus::FLUSHED;
				flush_.trigger(f, c.first);
				if (f.pool()) f.pool()->flush_.trigger(f, c.first);
			}
		} else if (c.second == ftl::data::ChangeType::FOREIGN) {
			auto &d = f._getData(c.first);
			if (d.status == ftl::data::ChannelStatus::DISPATCHED) {
				d.status = ftl::data::ChannelStatus::FLUSHED;
				flush_.trigger(f, c.first);
				if (f.pool()) f.pool()->flush_.trigger(f, c.first);
			}
		}
	}
}

void Session::flush(Frame &f, ftl::codecs::Channel c) {
	auto cc = f.changed_[c];
	if (cc == ftl::data::ChangeType::PRIMARY || cc == ftl::data::ChangeType::RESPONSE) {
		auto &d = f._getData(c);
		if (d.status == ftl::data::ChannelStatus::VALID) {
			d.status = ftl::data::ChannelStatus::FLUSHED;
			flush_.trigger(f, c);
			if (f.pool()) f.pool()->flush_.trigger(f, c);
		}
	} else if (cc == ftl::data::ChangeType::FOREIGN) {
		auto &d = f._getData(c);
		if (d.status == ftl::data::ChannelStatus::DISPATCHED) {
			d.status = ftl::data::ChannelStatus::FLUSHED;
			flush_.trigger(f, c);
			if (f.pool()) f.pool()->flush_.trigger(f, c);
		}
	}
}
