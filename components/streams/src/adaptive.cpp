#include "adaptive.hpp"

#include <loguru.hpp>

using ftl::stream::AdaptiveBitrate;
using ftl::stream::ABRState;
using ftl::stream::ABRNetworkStatus;

ABRNetworkStatus AdaptiveBitrate::status(int t_frame, int t_recv) {
	diff_error += t_recv - t_frame;
	if (diff_error < 0) diff_error = 0;

	float avgdiff = avg_err_(diff_error);
	
	int change_dir = 0;
	if (avgdiff > previous_diff_) change_dir = 1;
	else if (avgdiff < previous_diff_) change_dir = -1;
	float changer = avg_change_(change_dir);
	previous_diff_ = avgdiff;

	//LOG(INFO) << "Status measure: " << avgdiff << ", " << diff_error << ", " << changer;

	if (next_status_-- > 0) return ABRNetworkStatus::Pending;

	if ((changer == 1.0f && avgdiff >= 100.0f) || avgdiff > 200.0f) {
		next_status_ = 20;
		return ABRNetworkStatus::Degrading;
	} else if (changer < 0.0f && avgdiff >= 100.0f) {
		next_status_ = 5;
		return ABRNetworkStatus::Improving;
	} else if (avgdiff > 50.0f) {
		return ABRNetworkStatus::Pending;
	}

	return ABRNetworkStatus::Stable;
}

ABRState AdaptiveBitrate::nextState() {
	return ABRState::Maintain;
}

int AdaptiveBitrate::adjustment(int t_frame, int t_recv, int cur_rate) {

	auto s = status(t_frame, t_recv);

	if (s == ABRNetworkStatus::Degrading) {
		stable_ = false;
		if (last_increase_ > 0) known_unstable_ = std::min(known_unstable_, bitrate_);
		if (known_stable_ >= bitrate_) known_stable_ = std::max(0, bitrate_-10);
		bitrate_ = std::max(0, (last_increase_ > 0) ? bitrate_ - (2*last_increase_) : bitrate_/2);
		LOG(INFO) << "Degrade to " << bitrate_;
		last_increase_ = 0;
	}
	
	if (s == ABRNetworkStatus::Stable) {
		++stable_count_;
	} else {
		stable_count_ = 0;
	}

	if (stable_count_ >= ((stable_) ? 400 : 100)) {
		stable_count_ = 0;
		known_stable_ = std::max(known_stable_, bitrate_);

		if (known_stable_ < known_unstable_) {
			if (known_unstable_ - known_stable_ > 10) {
				bitrate_ += 10;
				last_increase_ = 10;
			} else if (known_unstable_ - known_stable_ > 2) {
				bitrate_ += 2;
				last_increase_ = 2;
			} else {
				known_unstable_ += 2;
				last_increase_ = std::max(0, known_stable_ - bitrate_);
				LOG(INFO) << "JUMP TO STABLE 1";
				stable_ = true;
				bitrate_ = known_stable_;
			}
		} else if (known_unstable_ < known_stable_) {
			known_unstable_ += 2;
			last_increase_ = std::max(0, known_stable_ - bitrate_);
			LOG(INFO) << "JUMP TO STABLE 2";
			bitrate_ += last_increase_;
		} else {
			last_increase_ = 2;
			bitrate_ = known_stable_+2;
		}

		if (last_increase_ > 0) LOG(INFO) << "Bitrate increase by " << last_increase_ << " to " << bitrate_;
	}

	// Needs a mode
	// First undo last change if incrementing, and then retry with smaller increment
	// Need to wait after drop to work through the delayed buffer.
	// If not working after N frames, decrement again
	// Maintain record of max stable rate so far, if increasing causes problem then
	// rapidly decrease and attempt to return to previous known stable position.
	// If returning to known stable causes problems again, decrement known stable and try again.

	/*if (roll_ratio > 60.0f) {
		bitrate_ = std::max(0, bitrate_-20);
	} else if (roll_ratio > 30.0f) {
		bitrate_ = std::max(0, bitrate_-2);
	} else if (roll_ratio < 5.0f && cur_rate == bitrate_) {
		bitrate_ = std::min(255, bitrate_+10);
	} else if (roll_ratio < 10.0f && cur_rate == bitrate_) {
		bitrate_ = std::min(255, bitrate_+2);
	}*/

	/*if (next_adjustment_-- <= 0) {
		if (roll_ratio < 1.0f) {
			bitrate_ = std::max(0, cur_rate-10);
			LOG(INFO) << "Fast decrease bitrate to " << int(bitrate_);
			pos_bitrate_ratio_ = 0;
			next_adjustment_ = 20;
		} else if (roll_ratio < 0.8f) {
			bitrate_ = std::max(0, cur_rate-2);
			LOG(INFO) << "Slow decrease bitrate to " << int(bitrate_);
			pos_bitrate_ratio_ = 0;
			next_adjustment_ = 6;
		} else if (roll_ratio > 2.0f) {
			bitrate_ = std::min(255, cur_rate+2);
			increase_max_ = bitrate_;
			LOG(INFO) << "Increase bitrate to " << int(bitrate_);
			pos_bitrate_ratio_ = 0;
			next_adjustment_ = 20;
		} else {
			pos_bitrate_ratio_ = 0;
		}
	}*/
	//LOG(INFO) << "Bandwidth Ratio = " << roll_ratio << " (" << bitrate_ << ")";

	bitrate_ = std::min(bitrate_, max_rate_);
	return bitrate_;
}

