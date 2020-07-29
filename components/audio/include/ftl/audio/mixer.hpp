#ifndef _FTL_AUDIO_MIXER_HPP_
#define _FTL_AUDIO_MIXER_HPP_

#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include "buffer.hpp"

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
class FixedMixer : public ftl::audio::Buffer<T> {
	public:
	FixedMixer() : Buffer<T>(CHAN, FRAME, 44100), write_position_(0), read_position_(0), offset_(0) { resize(1); }
	explicit FixedMixer(int tracks) : Buffer<T>(CHAN, FRAME, 44100), write_position_(0), read_position_(0), offset_(0) { resize(tracks); }
	FixedMixer(int tracks, int rate) : Buffer<T>(CHAN, FRAME, rate), write_position_(0), read_position_(0), offset_(0) { resize(tracks); }


	inline int maxFrames() const { return SIZE; }

	inline void readFrame(T *d) {
		T *out = d;
		if (read_position_ >= write_position_) {
			for (size_t i=0; i<CHAN*FRAME; ++i) *out++ = 0;
		} else {
			T *in = data_[(read_position_++) % SIZE];
			for (size_t i=0; i<CHAN*FRAME; ++i) *out++ = *in++;
		}
	}

	int size() const override { return (read_position_>=0) ? write_position_ - read_position_ : 0; }
	int frames() const override { return (read_position_>=0) ? write_position_ - read_position_ : 0; }

	/**
	 * Append sound samples to the end of the buffer. The samples may be over
	 * or under sampled so as to gradually introduce or remove a requested
	 * delay and hence change the latency of the audio.
	 */
	void write(const std::vector<T> &in) override;

	inline void write(int track, const std::vector<T> &in) {
		tracks_[track].write(in);
	}

	void mix();

	void read(std::vector<T> &out, int frames) override;

	void reset() override {
		Buffer<T>::reset();
		write_position_ = 0; //int(this->cur_delay_);
		read_position_ = 0;
	}

	inline int writePosition() const { return write_position_; }
	inline int readPosition() const { return read_position_; }
	inline int tracks() const { return track_num_; }

	inline void setDelay(int track, float d) { tracks_[track].setDelay(d); }
	inline float delay(int track) const { return tracks_[track].delay(); }

	inline void setGain(float g) { gain_ = g; }
	inline float gain() const { return gain_; }

	void resize(int tracks);

	private:
	int track_num_=0;
	int write_position_;
	int read_position_;
	int offset_;
	float gain_ = 1.0f;
	alignas(32) T data_[SIZE][CHAN*FRAME];
	std::vector<ftl::audio::FixedBuffer<T,CHAN,FRAME,SIZE>> tracks_;
};

// ==== Implementations ========================================================

template <typename T, int CHAN, int FRAME, int SIZE>
void FixedMixer<T,CHAN,FRAME,SIZE>::write(const std::vector<T> &in) {
	// Not supported...
}

template <typename T, int CHAN, int FRAME, int SIZE>
void FixedMixer<T,CHAN,FRAME,SIZE>::mix() {
	// Add together up to most recent frame
	int min_write = std::numeric_limits<int>::max();
	for (auto &t : tracks_) {
		min_write = std::min(t.writePosition(), min_write);
	}

	// For each frame
	while (write_position_ < min_write) {
		int wp = write_position_ % SIZE;
		float *ptr1 = data_[wp];

		// For each block of 8 float samples
		for (size_t i=0; i<CHAN*FRAME; i+=8) {
			Eigen::Map<Eigen::Matrix<float,8,1>,Eigen::Aligned32> v1(ptr1+i);
			v1.setZero();

			// For each track, accumulate the samples
			for (auto &t : tracks_) {
				const Eigen::Map<Eigen::Matrix<float,8,1>,Eigen::Aligned32> v2(&t.data(wp)[i]);
				v1 += v2;
			}

			v1 *= gain_;
		}

		++write_position_;
	}
}

template <typename T, int CHAN, int FRAME, int SIZE>
void FixedMixer<T,CHAN,FRAME,SIZE>::read(std::vector<T> &out, int count) {
	out.resize(FRAME*count*CHAN);
	T *ptr = out.data();
	for (int i=0; i<count; ++i) {
		readFrame(ptr);
		ptr += FRAME*CHAN;
	}
}

template <typename T, int CHAN, int FRAME, int SIZE>
void FixedMixer<T,CHAN,FRAME,SIZE>::resize(int t) {
	if (track_num_ == t) return;
	
	track_num_ = t;
	tracks_.reserve(t);
	while (static_cast<int>(tracks_.size()) < t) {
		auto &tr = tracks_.emplace_back();
		tr.setWritePosition(write_position_);
	}
}

// ==== Common forms ===========================================================

template <int SIZE>
using StereoMixerF = ftl::audio::FixedMixer<float,2,960,SIZE>;

template <int SIZE>
using MonoMixerF = ftl::audio::FixedMixer<float,1,960,SIZE>;

}
}

#endif  // _FTL_AUDIO_BUFFER_HPP_
