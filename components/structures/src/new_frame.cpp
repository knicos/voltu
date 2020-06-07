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
