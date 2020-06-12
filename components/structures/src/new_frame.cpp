#include <ftl/data/new_frame.hpp>
#include <ftl/data/framepool.hpp>

using ftl::data::Frame;
using ftl::data::Session;
using ftl::data::ChannelConfig;
using ftl::data::StorageMode;
using ftl::data::FrameStatus;

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

static std::unordered_map<ftl::codecs::Channel, ChannelConfig> reg_channels;

void ftl::data::registerChannel(ftl::codecs::Channel c, const ChannelConfig &config) {
	auto i = reg_channels.find(c);
	if (i != reg_channels.end()) {
		throw FTL_Error("Channel " << static_cast<unsigned int>(c) << " already registered");
	}

	reg_channels[c] = config;
}

void ftl::data::clearRegistry() {
	reg_channels.clear();
}

bool ftl::data::isPersistent(ftl::codecs::Channel c) {
	auto i = reg_channels.find(c);
	return (i != reg_channels.end()) ? i->second.mode == StorageMode::PERSISTENT : false;
}

bool ftl::data::isAggregate(ftl::codecs::Channel c) {
	auto i = reg_channels.find(c);
	return (i != reg_channels.end()) ? i->second.mode == StorageMode::AGGREGATE : false;
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

Frame::~Frame() {
	if (status_ == FrameStatus::CREATED) store();
	if (status_ == FrameStatus::STORED) flush();
	if (status_ != FrameStatus::RELEASED && pool_) pool_->release(*this);
};

bool ftl::data::Frame::has(ftl::codecs::Channel c) const {
	const auto &i = data_.find(c);
	if (i != data_.end() && i->second.status != ftl::data::ChannelStatus::INVALID) {
		return true;
	} else {
		return (parent_ && parent_->has(c)); 
	}
}

Frame::ChannelData &Frame::_getData(ftl::codecs::Channel c) {
	if (status_ == FrameStatus::RELEASED) throw FTL_Error("Reading a released frame");
	const auto &i = data_.find(c);
	if (i != data_.end()) {
		return i->second;
	} else if (parent_) {
		return parent_->_getData(c);
	} else throw FTL_Error("Missing channel (" << static_cast<unsigned int>(c) << ")");
}

const Frame::ChannelData &Frame::_getData(ftl::codecs::Channel c) const {
	if (status_ == FrameStatus::RELEASED) throw FTL_Error("Reading a released frame");
	const auto &i = data_.find(c);
	if (i != data_.end()) {
		return i->second;
	} else if (parent_) {
		return parent_->_getData(c);
	} else throw FTL_Error("Missing channel (" << static_cast<unsigned int>(c) << ")");
}

std::any &Frame::createAnyChange(ftl::codecs::Channel c, ftl::data::ChangeType t) {
	if (status_ != FrameStatus::CREATED) throw FTL_Error("Cannot apply change after store " << static_cast<int>(status_));

	auto &d = data_[c];
	if (d.status != ftl::data::ChannelStatus::FLUSHED) {
		d.status = ftl::data::ChannelStatus::DISPATCHED;
		d.encoded.clear();
		touch(c, t);
		return d.data;
	} else {
		throw FTL_Error("Channel is flushed and read-only: " << static_cast<unsigned int>(c));
	}
}

std::any &Frame::createAnyChange(ftl::codecs::Channel c, ftl::data::ChangeType t, std::vector<uint8_t> &data) {
	if (status_ != FrameStatus::CREATED) throw FTL_Error("Cannot apply change after store " << static_cast<int>(status_));

	auto &d = data_[c];
	if (d.status != ftl::data::ChannelStatus::FLUSHED) {
		d.status = ftl::data::ChannelStatus::DISPATCHED;
		d.encoded = std::move(data);
		touch(c, t);
		return d.data;
	} else {
		throw FTL_Error("Channel is flushed and read-only: " << static_cast<unsigned int>(c));
	}
}

std::any &Frame::createAny(ftl::codecs::Channel c) {
	if (status_ != FrameStatus::STORED) throw FTL_Error("Cannot create before store or after flush");

	auto &d = data_[c];
	if (d.status != ftl::data::ChannelStatus::FLUSHED) {
		d.status = ftl::data::ChannelStatus::VALID;
		d.encoded.clear();
		touch(c);
		return d.data;
	} else {
		throw FTL_Error("Channel is flushed and read-only: " << static_cast<unsigned int>(c));
	}
}

std::any &Frame::getAnyMutable(ftl::codecs::Channel c) {
	auto &d = _getData(c);
	return d.data;
}

const std::vector<uint8_t> &ftl::data::Frame::getEncoded(ftl::codecs::Channel c) const {
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

	for (auto c : changed_) {
		if (ftl::data::isPersistent(c.first) && hasOwn(c.first)) {
			auto &d = data_[c.first];
			auto &pd = parent_->data_[c.first];
			pd.data = d.data;
			//pd.encoded = std::move(d.encoded);
			pd.status = ChannelStatus::VALID;
			//data_.erase(c.first);
		}

		parent_->change_.trigger(*this, c.first);
		uint64_t sig = (uint64_t(id) << 32) + static_cast<unsigned int>(c.first);
		const auto &i = parent_->change_channel_.find(sig);
		if (i != parent_->change_channel_.end()) i->second.trigger(*this, c.first);
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
}

void Frame::moveTo(Frame &f) {
	if (status_ == FrameStatus::RELEASED) throw FTL_Error("Moving released frame");
	f.status_ = status_;
	f.parent_ = parent_;
	f.pool_ = pool_;
	f.data_ = std::move(data_);
	f.changed_ = std::move(changed_);
	status_ = FrameStatus::RELEASED;
}

void Frame::swapChanged(Frame &f) {
	for (auto x : changed_) {
		f.data_[x.first].data.swap(data_[x.first].data);
		f.changed_[x.first] = ftl::data::ChangeType::LOCAL;
	}
}

void Frame::swapChannel(ftl::codecs::Channel c, Frame &f) {
	if (f.hasOwn(c)) {
		auto &d = data_[c];
		auto &fd = f.data_[c];
		fd.data.swap(d.data);
		d.status = ftl::data::ChannelStatus::VALID;
		changed_[c] = f.changed_[c];
		f.changed_[c] = ftl::data::ChangeType::LOCAL;
	}
}

void Frame::reset() {
	for (auto &d : data_) {
		d.second.status = ChannelStatus::INVALID;
	}
	changed_.clear();
	status_ = FrameStatus::CREATED;
}

void Frame::hardReset() {
	status_ = FrameStatus::CREATED;
	changed_.clear();
	data_.clear();
}

// ==== Session ================================================================

ftl::Handle Session::onChange(uint32_t id, ftl::codecs::Channel c, const std::function<bool(Frame&,ftl::codecs::Channel)> &cb) {
	uint64_t sig = (uint64_t(id) << 32) + static_cast<unsigned int>(c);
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
	// TODO: Lock
	for (auto c : f.changed()) {
		if (c.second == ftl::data::ChangeType::LOCAL) {
			auto &d = f._getData(c.first);
			if (d.status == ftl::data::ChannelStatus::VALID) {
				d.status = ftl::data::ChannelStatus::FLUSHED;
				flush_.trigger(f, c.first);
			}
		} else if (c.second == ftl::data::ChangeType::FOREIGN) {
			auto &d = f._getData(c.first);
			if (d.status == ftl::data::ChannelStatus::DISPATCHED) {
				d.status = ftl::data::ChannelStatus::FLUSHED;
				flush_.trigger(f, c.first);
			}
		}
	}
}

void Session::flush(Frame &f, ftl::codecs::Channel c) {
	// TODO: Lock
	auto cc = f.changed_[c];
	if (cc == ftl::data::ChangeType::LOCAL) {
		auto &d = f._getData(c);
		if (d.status == ftl::data::ChannelStatus::VALID) {
			d.status = ftl::data::ChannelStatus::FLUSHED;
			flush_.trigger(f, c);
		}
	} else if (cc == ftl::data::ChangeType::FOREIGN) {
		auto &d = f._getData(c);
		if (d.status == ftl::data::ChannelStatus::DISPATCHED) {
			d.status = ftl::data::ChannelStatus::FLUSHED;
			flush_.trigger(f, c);
		}
	}
}
