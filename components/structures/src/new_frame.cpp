#include <ftl/data/new_frame.hpp>

using ftl::data::Frame;

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

bool Frame::flush() {
	if (parent_) {
		for (auto c : changed_) {
			parent_->changed_.emplace(c);
			parent_->data_[c] = std::move(data_[c]);
			data_.erase(c);
		}
		parent_->flush();
	} else {
		for (auto c : changed_) {
			auto i = triggers_.find(c);
			if (i != triggers_.end()) {
				for (auto f : i->second) {
					try {
						f(*this, c);
					} catch (const std::exception &e) {
						LOG(ERROR) << "Exception in frame flush: " << e.what();
					}
				}
			}
		}
	}
	changed_.clear();
	return true;
}

void Frame::merge(Frame &f) {
	for (auto x : f) {
		data_[x.first] = std::move(x.second);
		touch(x.first);
	}
}

void Frame::swapTo(Frame &f) {
	for (auto x : *this) {
		f.data_[x.first].swap(x.second);
		f.changed_.emplace(x.first);
		changed_.emplace(x.first);
	}
}

void Frame::swapChanged(Frame &f) {
	for (auto x : changed_) {
		f.data_[x].swap(data_[x]);
		f.changed_.emplace(x);
	}
}

void Frame::swapChannel(ftl::codecs::Channel c, Frame &f) {
	if (has(c)) {
		f.data_[c].swap(data_[c]);
		f.changed_.emplace(c);
		changed_.emplace(c);
	}
}

void Frame::clear() {
	changed_.clear();
	data_.clear();
}
