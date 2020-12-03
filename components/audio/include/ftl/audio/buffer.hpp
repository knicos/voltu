/**
 * @file buffer.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_AUDIO_BUFFER_HPP_
#define _FTL_AUDIO_BUFFER_HPP_

#include <vector>
#include <cmath>
#include <Eigen/Eigen>

namespace ftl {
namespace audio {

/**
 * Generic audio frame buffer abstract class. Classes implementing this allow
 * asynchronous read and write of audio frames with an adjustable delay to
 * change the amount of data being buffered. If writes cannot keep up with
 * reads, or are bursty in nature, then more buffering is required.
 */
template <typename T>
class Buffer {
	public:
	typedef T type;

	Buffer(int channels, int framesize, int rate) : rate_(rate), cur_delay_(0.0f), req_delay_(0.0f), channels_(channels), frame_size_(framesize) {}
	virtual ~Buffer() {}

	/** Write one or more audio frames. */
	virtual void write(const std::vector<T> &in)=0;

	/** Read a given number of audio frames. */
	virtual void read(std::vector<T> &out, int)=0;

	/** Either 1 for mono, or 2 for stereo. */
	inline int channels() const { return channels_; }

	/** Number of samples per frame. */
	inline int frameSize() const { return frame_size_; }

	/** Number of samples per second. */
	inline int sampleRate() const { return rate_; }

	/**
	 * Change the buffering delay.
	 * @param d Seconds
	 */
	void setDelay(float d) {
		req_delay_ = d  * static_cast<float>(rate_);
		// Big jumps should be instant
		if (fabs(req_delay_ - cur_delay_) > 0.5f*static_cast<float>(rate_)) {
			//cur_delay_ = req_delay_;
			reset();
		}
	}

	/** Get the current buffering delay in seconds. */
	float delay() const { return cur_delay_ / static_cast<float>(rate_); }

	inline void setGain(float g) { gain_ = g; }
	inline float gain() const { return gain_; }

	virtual void reset() {
		cur_delay_ = req_delay_;
	}

	virtual int size() const=0;
	virtual int frames() const=0;

	protected:
	int rate_;
	float cur_delay_;
	float req_delay_;
	int channels_;
	int frame_size_;
	float gain_ = 1.0f;
};

//static constexpr int kBufferCount = 100;

/**
 * A fast circular buffer to capture, play and manipulate audio data.
 * This class can be used directly with portaudio. The hardware uses
 * `readFrame` and `writeFrame` to consume or append audio data. A more
 * advanced `write` function allows for non-frame aligned data and for time
 * dilation / shifting, and amplitude control.
 */
template <typename T, int CHAN, int FRAME, int SIZE>
class FixedBuffer : public ftl::audio::Buffer<T> {
	public:
	FixedBuffer() : Buffer<T>(CHAN, FRAME, 44100), write_position_(0), read_position_(-1), offset_(0) {}
	explicit FixedBuffer(int rate) : Buffer<T>(CHAN, FRAME, rate), write_position_(0), read_position_(-1),
			offset_(0) {}


	inline int maxFrames() const { return SIZE; }

	/**
	 * Fast safe single frame write. Used by audio interrupt.
	 */
	inline void writeFrame(const T *d) {
		const T *in = d;
		T *out = data_[(write_position_++) % SIZE];
		for (size_t i=0; i<CHAN*FRAME; ++i) *out++ = *in++;
		if (write_position_ > 5 && read_position_ < 0) read_position_ = 0;
	}

	/**
	 * Fast safe single frame read. Used by audio interrupt.
	 */
	inline void readFrame(T *d) {
		T* __restrict out = d;

		if (read_position_ < 0 || read_position_ >= write_position_-1) {
			for (size_t i=0; i<CHAN*FRAME; ++i) *out++ = 0;
		} else {
			const T* __restrict in = data_[(read_position_++) % SIZE];

			// 16 byte aligned, use SIMD intrinsics
			if ((size_t(out) & 0xf) == 0) {
				for (size_t i=0; i<CHAN*FRAME; i += 4) {
					Eigen::Map<Eigen::Matrix<float,4,1>,Eigen::Aligned16> vout(out+i);
					const Eigen::Map<const Eigen::Matrix<float,4,1>,Eigen::Aligned16> vin(in+i);
					vout = vin*this->gain_;
				}
			// Not aligned
			} else {
				for (size_t i=0; i<CHAN*FRAME; ++i) *out++ = this->gain_ * (*in++);
			}
		}
	}

	int size() const override { return (read_position_>=0) ? write_position_ - 2 - read_position_ : 0; }
	int frames() const override { return (read_position_>=0) ? write_position_ - 2 - read_position_ : 0; }

	/**
	 * Append sound samples to the end of the buffer. The samples may be over
	 * or under sampled so as to gradually introduce or remove a requested
	 * delay and hence change the latency of the audio.
	 */
	void write(const std::vector<T> &in) override;

	void read(std::vector<T> &out, int frames) override;

	/**
	 * Clear all buffer data.
	 */
	void reset() override {
		Buffer<T>::reset();
		write_position_ = 0; //int(this->cur_delay_);
		read_position_ = 0;
	}

	inline T *data() { return (T*)data_; }
	inline T *data(int f) { return data_[f]; }

	inline int writePosition() const { return write_position_; }
	inline void setWritePosition(int p) { write_position_ = p; }

	private:
	int write_position_;
	int read_position_;
	int offset_;
	alignas(32) T data_[SIZE][CHAN*FRAME];
};

// ==== Implementations ========================================================

template <typename T, int CHAN>
static T fracIndex(const std::vector<T> &in, float ix, int c) {
	const auto i1 = static_cast<unsigned int>(ix);
	const auto i2 = static_cast<unsigned int>(ix+1.0f);
	const float alpha = ix - static_cast<float>(i1);
	return static_cast<T>((i2*CHAN+CHAN >= in.size()) ? in[i1*CHAN+c] : in[i1*CHAN+c]*(1.0f-alpha) + in[i2*CHAN+c]*alpha);
}

inline float clamp(float v, float c) { return (v < -c) ? -c : (v > c) ? c : v; }

template <typename T, int CHAN, int FRAME, int SIZE>
void FixedBuffer<T,CHAN,FRAME,SIZE>::write(const std::vector<T> &in) {
	float i=0.0f;
	float s = static_cast<float>(in.size()) / static_cast<float>(CHAN);
	
	while (i <= s-1.0f) {
		T *ptr = data_[write_position_ % SIZE]+offset_;
		
		for (int c=0; c<CHAN; ++c) *ptr++ = fracIndex<T,CHAN>(in, i, c);

		const float d = 0.6f*clamp((this->req_delay_ - this->cur_delay_) / static_cast<float>(this->rate_), 0.5f);
		i += 1.0f - d;
		this->cur_delay_ += d;

		offset_+= CHAN;
		if (offset_ == CHAN*FRAME) {
			offset_ = 0;
			++write_position_;
		}
	}
	if (write_position_ > 5 && read_position_ < 0) read_position_ = 0;
}

template <typename T, int CHAN, int FRAME, int SIZE>
void FixedBuffer<T,CHAN,FRAME,SIZE>::read(std::vector<T> &out, int count) {
	out.resize(FRAME*count*CHAN);
	T *ptr = out.data();
	for (int i=0; i<count; ++i) {
		readFrame(ptr);
		ptr += FRAME*CHAN;
	}
}

// ==== Common forms ===========================================================

template <int SIZE>
using StereoBuffer16 = ftl::audio::FixedBuffer<short,2,960,SIZE>;

template <int SIZE>
using MonoBuffer16 = ftl::audio::FixedBuffer<short,1,960,SIZE>;

template <int SIZE>
using StereoBufferF = ftl::audio::FixedBuffer<float,2,960,SIZE>;

template <int SIZE>
using MonoBufferF = ftl::audio::FixedBuffer<float,1,960,SIZE>;

}
}

#endif  // _FTL_AUDIO_BUFFER_HPP_
