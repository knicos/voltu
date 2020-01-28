#ifndef _FTL_AUDIO_BUFFER_HPP_
#define _FTL_AUDIO_BUFFER_HPP_

#include <vector>

namespace ftl {
namespace audio {

//static constexpr int kBufferCount = 100;

/**
 * A fast circular buffer to capture, play and manipulate audio data.
 * This class can be used directly with portaudio. The hardware uses
 * `readFrame` and `writeFrame` to consume or append audio data. A more
 * advanced `write` function allows for non-frame aligned data and for time
 * dilation / shifting, and amplitude control.
 */
template <typename T, int CHAN, int FRAME, int SIZE>
class FixedBuffer {
	public:
	typedef T type;

	FixedBuffer() : write_position_(0), read_position_(-1), offset_(0), rate_(44100),
			cur_delay_(0.0f), req_delay_(0.0f) {}
	explicit FixedBuffer(int rate) : write_position_(0), read_position_(-1),
			offset_(0), rate_(rate), cur_delay_(0.0f), req_delay_(0.0f) {}

	int sampleRate() const { return rate_; }

	inline int channels() const { return CHAN; }
	inline int frameSize() const { return FRAME; }
	inline int maxFrames() const { return SIZE; }

	void setDelay(float d) {
		req_delay_ = d  * static_cast<float>(rate_);
	}

	float delay() const { return cur_delay_ / static_cast<float>(rate_); }

	inline void writeFrame(const T *d) {
		const T *in = d;
		T *out = &data_[(write_position_++) % SIZE][0];
		for (size_t i=0; i<CHAN*FRAME; ++i) *out++ = *in++;
		if (write_position_ > 5 && read_position_ < 0) read_position_ = 0;
	}

	inline void readFrame(T *d) {
		T *out = d;
		if (read_position_ < 0 || read_position_ >= write_position_-1) {
			for (size_t i=0; i<CHAN*FRAME; ++i) *out++ = 0;
		} else {
			T *in = &data_[(read_position_++) % SIZE][0];
			for (size_t i=0; i<CHAN*FRAME; ++i) *out++ = *in++;
		}
	}

	int size() const { return (read_position_>=0) ? write_position_ - 2 - read_position_ : 0; }
	int frames() const { return (read_position_>=0) ? write_position_ - 2 - read_position_ : 0; }

	/**
	 * Append sound samples to the end of the buffer. The samples may be over
	 * or under sampled so as to gradually introduce or remove a requested
	 * delay and hence change the latency of the audio.
	 */
	void write(const std::vector<T> &in);

	private:
	int write_position_;
	int read_position_;
	int offset_;
	T data_[SIZE][CHAN*FRAME];
	int rate_;

	float cur_delay_;
	float req_delay_;
};

// ==== Implementations ========================================================

template <typename T, int CHAN>
static T fracIndex(const std::vector<T> &in, float ix, int c) {
	const auto i1 = static_cast<unsigned int>(ix);
	const auto i2 = static_cast<unsigned int>(ix+1.0f);
	const float alpha = ix - static_cast<float>(i1);
	return (i2*CHAN+CHAN >= in.size()) ? in[i1*CHAN+c] : in[i1*CHAN+c]*(1.0f-alpha) + in[i2*CHAN+c]*alpha;
}

inline float clamp(float v, float c) { return (v < -c) ? -c : (v > c) ? c : v; }

template <typename T, int CHAN, int FRAME, int SIZE>
void FixedBuffer<T,CHAN,FRAME,SIZE>::write(const std::vector<T> &in) {
	float i=0.0f;
	float s = static_cast<float>(in.size()) / static_cast<float>(CHAN);
	
	while (i <= s-1.0f) {
		T *ptr = data_[write_position_ % SIZE]+offset_;
		
		for (int c=0; c<CHAN; ++c) *ptr++ = fracIndex<T,CHAN>(in, i, c);

		const float d = 0.6f*clamp((req_delay_ - cur_delay_) / static_cast<float>(rate_), 0.5f);
		i += 1.0f - d;  // FIXME: Is this correct? Seems to function but perhaps not ideal

		/*if (d > 0.0f) {	// Increase delay = oversample with increment < 1.0
			//i += 1.0f * (1.0f - d);
			i += 1.0f - d;
		} else {		// Decrease delay = undersample with increment > 1.0
			//i += 1.0f / (1.0f + d);
			i += 1.0f - d;
		}*/
		cur_delay_ += d;

		offset_+= CHAN;
		if (offset_ == CHAN*FRAME) {
			offset_ = 0;
			++write_position_;
		}
	}
	if (write_position_ > 20 && read_position_ < 0) read_position_ = 0;
}

// ==== Common forms ===========================================================

template <int SIZE>
using StereoBuffer16 = ftl::audio::FixedBuffer<short,2,256,SIZE>;

template <int SIZE>
using MonoBuffer16 = ftl::audio::FixedBuffer<short,1,256,SIZE>;

}
}

#endif  // _FTL_AUDIO_BUFFER_HPP_
