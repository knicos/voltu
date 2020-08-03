#ifndef _FTL_STREAM_ADAPTIVE_HPP_
#define _FTL_STREAM_ADAPTIVE_HPP_

#include <ftl/utility/rollingavg.hpp>

namespace ftl {
namespace stream {

enum class ABRNetworkStatus {
	Stable=0,	// Appears stable but not room to increase much
	Improving,	// Going in correct direction but not resolved
	Pending,	// Not enough information yet
	Degrading,	// Urgently decrease bitrate
	Good		// Could potentially increase bitrate
};

enum class ABRState {
	Unknown,				// Current network conditions unknown
	Increased_Recover,		// Moving back to past stable point
	Increased_Wait,			// Gentle increase, wait for network status
	Maintain,				// Stay at current rate for a while
	Decreased_Wait,			// Decrease and wait for network status
	Decreased_50_Wait,		// Rapid decrease and move to recover
};

class AdaptiveBitrate {
	public:
	explicit AdaptiveBitrate(int initial) : bitrate_(initial) {}

	inline void setMaxRate(int m) { max_rate_ = m; };

	inline int current() const { return bitrate_; }

	int adjustment(int t_frame, int t_recv, int rec_rate);

	ABRNetworkStatus status(int, int);

	ABRState nextState();

	private:
	int max_rate_=200;
	int bitrate_=32;
	int increase_max_=-1;
	int diff_error=0;
	float previous_diff_=0.0f;
	int next_status_=10;
	int known_stable_=-1;
	int known_unstable_=255;
	int stable_count_=0;
	int last_increase_=0;
	bool stable_=false;

	ftl::utility::RollingAvg<int, 8u> avg_err_;
	ftl::utility::RollingAvg<int, 8u> avg_ahead_;
	ftl::utility::RollingAvg<float, 4u> avg_change_;

	int pos_bitrate_ratio_ = 0;
	int delay_bitrate_increase_ = 20;
	int next_adjustment_=0;
};

}
}

#endif
