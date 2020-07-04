#ifndef _FTL_LIBSTEREO_COSTS_CENSUS_HPP_
#define _FTL_LIBSTEREO_COSTS_CENSUS_HPP_

#include <opencv2/core/core.hpp>
#include "array2d.hpp"
#include "dsbase.hpp"
#include <stereo_types.hpp>
#include <cuda_runtime.h>

namespace impl {
	__host__ __device__ static inline uint64_t popcount(const uint64_t bits) {
		#if defined(__CUDA_ARCH__)
			return __popcll(bits);
		#elif defined(_MSC_VER)
			return __popcnt64(bits);
		#elif defined(__GNUC__)
			return __builtin_popcountl(bits);
		#else
			static_assert(false, "unsupported compiler (popcount intrinsic)");
		#endif
	}

	/**
	 * Hamming cost, template parameter number of bits
	 */
	template<int SIZE>
	struct HammingCost : DSImplBase<unsigned short> {
		typedef unsigned short Type;

		HammingCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<Type>({w,h,dmin,dmax}) {}
		HammingCost() : DSImplBase<Type>({0,0,0,0}) {}

		__host__ __device__ inline Type operator()(const int y, const int x, const int d) const {
			if ((x-d) < 0) { return COST_MAX; }
			unsigned short c = 0;

			#pragma unroll
			for (int i = 0; i < WSTEP; i++) {
				c+= popcount(l(y, x*WSTEP+i) ^ r(y, (x-d)*WSTEP+i));
			}
			return c;
		}

		// number of uint64_t values for each window
		static constexpr int WSTEP = (SIZE - 1)/(sizeof(uint64_t)*8) + 1;
		static constexpr Type COST_MAX = SIZE;

		Array2D<uint64_t>::Data l;
		Array2D<uint64_t>::Data r;
	};

	template<uint8_t WW, uint8_t WH, int BPP=1>
	using CensusMatchingCost = HammingCost<WW*WH*BPP>;

	/**
	 * Normalized Hamming cost, same as above except float type and normalized
	 * by number of bits (user set). Cost will always be within range [0, 1].
	 */
	template<int SIZE>
	struct NormalizedHammingCost : DSImplBase<float> {
		static_assert(SIZE%64 == 0, "size must be multiple of 64");

		typedef float Type;

		NormalizedHammingCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<Type>({w,h,dmin,dmax}) {}
		NormalizedHammingCost() : DSImplBase<Type>({0,0,0,0}) {}

		__host__ __device__ inline Type operator()(const int y, const int x, const int d) const {
			if ((x-d) < 0) { return COST_MAX; }
			float c = 0;

			#pragma unroll
			for (int i = 0; i < WSTEP; i++) {
				c+= popcount(l(y, x*WSTEP+i) ^ r(y, (x-d)*WSTEP+i));
			}
			return c*normalize;
		}

		// number of uint64_t values for each window
		static constexpr int WSTEP = (SIZE - 1)/(sizeof(uint64_t)*8) + 1;
		static constexpr Type COST_MAX = 1.0f;

		Array2D<uint64_t>::Data l;
		Array2D<uint64_t>::Data r;
		float normalize = 1.0f; // set to 1.0f/(number of bits used)
	};

	template<uint8_t WW, uint8_t WH, int BPP=1>
	using NormalizedCensusMatchingCost = NormalizedHammingCost<WW*WH*BPP>;

	/**
	 * WeightedCensusMatchingCost
	 */
	template<uint8_t R, uint8_t NBINS>
	struct WeightedCensusMatchingCost : DSImplBase<unsigned short> {
		static_assert(R % 2 == 1, "R must be odd");
		typedef unsigned short Type;

		WeightedCensusMatchingCost() : DSImplBase<Type>({0,0,0,0}) {}
		WeightedCensusMatchingCost(ushort w, ushort h, ushort dmin, ushort dmax) :
				DSImplBase<unsigned short>({w,h,dmin,dmax}) {

			const float wbin = float(R)/(2.0f*float(NBINS)); // bin width
			int step = 0;
			int i = 0;

			for (int wy = -R/2; wy <= R/2; wy++) {
			for (int wx = -R/2; wx <= R/2; wx++) {
					const int bin = floor(sqrtf(wx*wx + wy*wy)/wbin);
					masks[step][bin] |= uint64_t(1) << i;

					i += 1;
					if (i % 64 == 0) { step++; i = 0; }
			}}

			float weight = 1.0;
			int bin = 0;
			do {
				weights[bin] = weight;
				weight *= W;
				bin++;
			} while(bin != NBINS);
		}

		WeightedCensusMatchingCost<R,NBINS> &operator=(const WeightedCensusMatchingCost<R,NBINS> &c) {
			ct_l = c.ct_l;
			ct_r = c.ct_r;
			W = c.W;
			memcpy(masks, c.masks, sizeof(masks));
			memcpy(weights, c.weights, sizeof(weights));
			return *this;
		}

		__host__ __device__ inline unsigned short operator()(const int y, const int x, const int d) const {
			if ((x-d) < 0) { return COST_MAX; }
			float c = 0;

			#pragma unroll
			for (int i = 0; i < WSTEP; i++) {
				# pragma unroll
				for (int bin = 0; bin < NBINS; bin++) {
					uint64_t bits = ct_l(y, x*WSTEP+i) ^ ct_r(y, (x-d)*WSTEP+i);
					bits &= masks[i][bin];
					c += float(popcount(bits)) * weights[bin];
				}
			}

			return round(c);
		}

		static constexpr int BPP = 1;
		// number of uint64_t values for each window
		static constexpr int WSTEP = (BPP*R*R - 1)/(sizeof(uint64_t)*8) + 1;
		static constexpr Type COST_MAX = R*R*BPP;

		uint64_t masks[WSTEP][NBINS] = {0};
		float weights[NBINS] = {0.0f};
		float W = 0.75f;

		Array2D<uint64_t>::Data ct_l;
		Array2D<uint64_t>::Data ct_r;
	};

	template<uint8_t WW, uint8_t WH, int BPP=1, bool RMASK=true>
	struct CensusMaskMatchingCost : DSImplBase<unsigned short> {
		typedef unsigned short Type;

		CensusMaskMatchingCost(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<unsigned short>({w,h,dmin,dmax}) {}

		__host__ __device__ inline unsigned short operator()(const int y, const int x, const int d) const {
			if ((x-d) < 0) { return COST_MAX; }
			unsigned short c = 0;

			#pragma unroll
			for (int i = 0; i < WSTEP; i++) {
				uint64_t mask = mask_l(y, x*WSTEP+i);
				if (RMASK) {
					/* intersection of l and r masks */
					mask &= mask_r(y, x*WSTEP+i);
				}

				const uint64_t bits =
					(ct_l(y, x*WSTEP+i) ^ ct_r(y, (x-d)*WSTEP+i)) & mask;

				c += popcount(bits);
			}
			return c;
		}

		// number of uint64_t values for each window
		static constexpr int WSTEP = (BPP*WW*WH - 1)/(sizeof(uint64_t)*8) + 1;
		static constexpr Type COST_MAX = WW*WH*BPP;

		Array2D<uint64_t>::Data ct_l;
		Array2D<uint64_t>::Data ct_r;
		Array2D<uint64_t>::Data mask_l;
		Array2D<uint64_t>::Data mask_r;
	};
}

class CensusMatchingCost : public DSBase<impl::CensusMatchingCost<9,7,1>> {
public:
	typedef impl::CensusMatchingCost<9,7,1> DataType;
	typedef unsigned short Type;

	CensusMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	CensusMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			ct_l_(width*data().WSTEP, height), ct_r_(width*data().WSTEP,height)
		{
			data().l = ct_l_.data();
			data().r = ct_r_.data();
		}

	inline void setPattern(CensusPattern p) { pattern_ = p; }

	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);
	void set(const Array2D<uchar> &l, const Array2D<uchar> &r, const Array2D<uchar> &hl, const Array2D<uchar> &hr);
	static constexpr Type COST_MAX = DataType::COST_MAX;

protected:
	Array2D<uint64_t> ct_l_;
	Array2D<uint64_t> ct_r_;
	CensusPattern pattern_ = CensusPattern::STANDARD;
};

class MiniCensusMatchingCost : public DSBase<impl::CensusMatchingCost<5,3,1>> {
public:
	typedef impl::CensusMatchingCost<5,3,1> DataType;
	typedef unsigned short Type;

	MiniCensusMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	MiniCensusMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			ct_l_(width*data().WSTEP, height), ct_r_(width*data().WSTEP,height)
		{
			data().l = ct_l_.data();
			data().r = ct_r_.data();
		}

	inline void setPattern(CensusPattern p) { pattern_ = p; }

	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);
	void set(const Array2D<uchar> &l, const Array2D<uchar> &r, const Array2D<uchar> &hl, const Array2D<uchar> &hr);
	static constexpr Type COST_MAX = DataType::COST_MAX;

protected:
	Array2D<uint64_t> ct_l_;
	Array2D<uint64_t> ct_r_;
	CensusPattern pattern_ = CensusPattern::STANDARD;
};

class ExpandingCensusMatchingCost : public DSBase<impl::WeightedCensusMatchingCost<9,3>> {
public:
	typedef impl::WeightedCensusMatchingCost<9,3> DataType;
	typedef unsigned short Type;

	ExpandingCensusMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	ExpandingCensusMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			ct_l_(width*data().WSTEP, height), ct_r_(width*data().WSTEP,height)
		{
			data().ct_l = ct_l_.data();
			data().ct_r = ct_r_.data();
		}

	inline void setPattern(CensusPattern p) { pattern_ = p; }

	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);
	//void set(const Array2D<uchar> &l, const Array2D<uchar> &r, const Array2D<uchar> &hl, const Array2D<uchar> &hr);
	static constexpr Type COST_MAX = DataType::COST_MAX;

protected:
	Array2D<uint64_t> ct_l_;
	Array2D<uint64_t> ct_r_;
	CensusPattern pattern_ = CensusPattern::STANDARD;
};

class WeightedCensusMatchingCost : public DSBase<impl::WeightedCensusMatchingCost<7, 3>> {
public:
	typedef impl::WeightedCensusMatchingCost<7, 3> DataType;
	typedef unsigned short Type;

	WeightedCensusMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	WeightedCensusMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			ct_l_(width*data().WSTEP, height), ct_r_(width*data().WSTEP,height)
		{
			data().ct_l = ct_l_.data();
			data().ct_r = ct_r_.data();
		}

	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uchar>& l, const Array2D<uchar>& r);
	void set(const Array2D<uchar> &l, const Array2D<uchar> &r, const Array2D<uchar> &hl, const Array2D<uchar> &hr);
	static constexpr Type COST_MAX = DataType::COST_MAX;

protected:
	Array2D<uint64_t> ct_l_;
	Array2D<uint64_t> ct_r_;
};

class CensusMaskMatchingCost : public DSBase<impl::CensusMaskMatchingCost<9,7,1>> {
public:
	typedef impl::CensusMaskMatchingCost<9,7,1> DataType;
	typedef unsigned short Type;

	CensusMaskMatchingCost() : DSBase<DataType>(0, 0, 0, 0) {};
	CensusMaskMatchingCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max),
			ct_l_(width*data().WSTEP, height), ct_r_(width*data().WSTEP,height),
			mask_l_(width*data().WSTEP, height), mask_r_(width*data().WSTEP,height)
		{
			data().ct_l = ct_l_.data();
			data().ct_r = ct_r_.data();
			data().mask_l = mask_l_.data();
			data().mask_r = mask_r_.data();
		}

	void set(cv::InputArray l, cv::InputArray r);
	void set(const Array2D<uint64_t>& l, const Array2D<uint64_t>& r);

	void setMask(cv::InputArray l, cv::InputArray r);
	void setMask(const Array2D<uchar>& l, const Array2D<uchar>& r);

	static constexpr Type COST_MAX = DataType::COST_MAX;

protected:
	Array2D<uint64_t> ct_l_;
	Array2D<uint64_t> ct_r_;
	Array2D<uint64_t> mask_l_;
	Array2D<uint64_t> mask_r_;
};

#endif
