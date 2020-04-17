#include "catch.hpp"
#include "../src/dsbase.hpp"
#include "../src/cost_aggregation.hpp"

#include <atomic>
#include <iostream>

template <typename DSIIN>
struct TestAggregateIncrement {
	typedef typename DSIIN::Type Type;
	typedef typename DSIIN::Type costtype_t;

	const DSIIN in;
	typename DisparitySpaceImage<costtype_t>::DataType out;

	struct PathData : BasePathData<costtype_t>  {
		// No extra data...
	};

	struct DirectionData {

	};

	void direction(DirectionData &data, DisparitySpaceImage<costtype_t> &buffer) {
		out = buffer.data();
	}

	__host__ __device__ inline void startPath(ushort2 pixel, ushort thread, ushort stride, PathData &data) {}
	__host__ __device__ inline void endPath(ushort2 pixel, ushort thread, ushort stride, PathData &data) {}

	__host__ __device__ inline void operator()(ushort2 pixel, ushort thread, ushort stride, PathData &data) {
		int x_prev = pixel.x - data.direction.x;
		int y_prev = pixel.y - data.direction.y;
		costtype_t c = 0;

		if (x_prev >= 0 && y_prev >= 0 && x_prev < out.width && y_prev < out.height) {
			c = out(y_prev,x_prev,in.disp_min) + 1;
		}

		const int d_stop = int(out.disp_max)-int(out.disp_min);

		for (int d=thread; d<=d_stop; d+=stride) {
			// for tests, each value updated exactly once (each path separately)
			// NOT reliable (threading)
			if (out(pixel.y,pixel.x,d+in.disp_min) != 0) { throw std::exception(); }

			out(pixel.y,pixel.x,d+in.disp_min) += c;
		}
	}

	/* __host__ __device__ void operator()(int x, int y, int i, const int dx, const int dy, PathData &data) {
		int x_prev = x - dx;
		int y_prev = y - dy;
		int c = 0;

		if (x_prev >= 0 && y_prev >= 0 && x_prev < out.width && y_prev < out.height) {
			c = out(y_prev,x_prev,in.disp_min) + 1;
		}
		for (int d=in.disp_min; d <= in.disp_max; d++) {
			if (out(y,x,d) != 0) { throw std::exception(); }
			out(y,x,d) += c;
		}
	}*/
};

struct DummyCostImpl : impl::DSImplBase<short> {
	typedef short Type;

	DummyCostImpl(ushort w, ushort h, ushort dmin, ushort dmax) : DSImplBase<short>({w,h,dmin,dmax}) {}

	__host__ __device__ inline short operator()(const int y, const int x, const int d) const {
		return 0;
	}
};

class DummyCost : public DSBase<DummyCostImpl> {
public:
	typedef DummyCostImpl DataType;
	typedef short Type;

	DummyCost() : DSBase<DataType>(0, 0, 0, 0) {};
	DummyCost(int width, int height, int disp_min, int disp_max)
		: DSBase<DataType>(width, height, disp_min, disp_max) {}
};

static void test_aggregation_increment(const int w, const int h, const int dmin, const int dmax) {
	const int size = w*h*(dmax-dmin+1);

	DummyCost dsi(w,h,dmin,dmax);
	TestAggregateIncrement<DummyCost::DataType> func = {dsi.data()};
	PathAggregator<TestAggregateIncrement<DummyCost::DataType>> aggr;

	SECTION("Left to right") {
		auto &out = aggr(func, AggregationDirections::LEFTRIGHT);
		int valid = 0;

		for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
		for (int d = dmin; d <= dmax; d++) {
			auto val = out(y,x,d);
			valid += (out(y,x,d) == x) ? 1 : 0;
		}}}

		REQUIRE(valid == size);
	}

	SECTION("Right to left") {
		auto &out = aggr(func, AggregationDirections::RIGHTLEFT);
		int valid = 0;

		for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
		for (int d = dmin; d <= dmax; d++) {
			valid += (out(y,x,d) == w-1-x) ? 1 : 0;
		}}}

		REQUIRE(valid == size);
	}


	SECTION("Top to bottom") {
		auto &out = aggr(func, AggregationDirections::UPDOWN);
		int valid = 0;

		for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
		for (int d = dmin; d <= dmax; d++) {
			valid += (out(y,x,d) == y) ? 1 : 0;
		}}}

		REQUIRE(valid == size);
	}

	SECTION("Bottom to top") {
		auto &out = aggr(func, AggregationDirections::DOWNUP);
		int valid = 0;

		for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
		for (int d = dmin; d <= dmax; d++) {
			valid += (out(y,x,d) == h-y-1) ? 1 : 0;
		}}}

		REQUIRE(valid == size);
	}

	SECTION("Top left to bottom right") {
		auto &out = aggr(func, AggregationDirections::TOPLEFTBOTTOMRIGHT);
		int valid = 0;

		for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
		for (int d = dmin; d <= dmax; d++) {
			valid += (out(y,x,d) == std::min(x,y)) ? 1 : 0;
		}}}

		REQUIRE(valid == size);
	}

	SECTION("Bottom right to top left") {
		auto &out = aggr(func, AggregationDirections::BOTTOMRIGHTTOPLEFT);
		int valid = 0;

		for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
		for (int d = dmin; d <= dmax; d++) {
			valid += (out(y,x,d) == std::min(w-1-x,h-1-y)) ? 1 : 0;
		}}}

		REQUIRE(valid == size);
	}

	SECTION("Bottom left to top right") {
		auto &out = aggr(func, AggregationDirections::BOTTOMLEFTTOPRIGHT);
		int valid = 0;

		for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
		for (int d = dmin; d <= dmax; d++) {
			valid += (out(y,x,d) == std::min(x,h-1-y)) ? 1 : 0;
		}}}

		REQUIRE(valid == size);
	}

	SECTION("Top right to bottom left") {
		auto &out = aggr(func, AggregationDirections::TOPRIGHTBOTTOMLEFT);
		int valid = 0;

		for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
		for (int d = dmin; d <= dmax; d++) {
			valid += (out(y,x,d) == std::min(w-1-x,y)) ? 1 : 0;
		}}}

		REQUIRE(valid == size);
	}
}

TEST_CASE("Increment paths [100x100]", "") {
	test_aggregation_increment(100, 100, 10, 20);
}

TEST_CASE("Increment paths [100x200]", "") {
	test_aggregation_increment(100, 200, 10, 20);
}

TEST_CASE("Increment paths [200x100]", "") {
	test_aggregation_increment(200, 100, 10, 20);
}

/*TEST_CASE("Vertical aggregation", "") {
	SECTION("Perform an aggregation") {
		CensusMatchingCost dsi(100,100,10,20,9,7);

		cv::Mat left(100,100, CV_8UC1);
		cv::Mat right(100,100, CV_8UC1);
		left.setTo(cv::Scalar(0));
		right.setTo(cv::Scalar(0));
		dsi.setLeft(left);
		dsi.setRight(right);

		DisparitySpaceImage<unsigned short> out(100,100,10,20);
		out.set(1);
		DisparitySpaceImage<unsigned short> previous(100,1,10,20);

		Aggregate<CensusMatchingCost, unsigned short> aggr = {0,0, out.data(), previous.data(), dsi.data()};
		aggregate_vertical_all(aggr);

		REQUIRE( out.data()(50,50,12) == 1 );
	}
}

TEST_CASE("Diagonal upper aggregation", "") {
	SECTION("Perform an aggregation") {
		CensusMatchingCost dsi(100,100,10,20,9,7);

		cv::Mat left(100,100, CV_8UC1);
		cv::Mat right(100,100, CV_8UC1);
		left.setTo(cv::Scalar(0));
		right.setTo(cv::Scalar(0));
		dsi.setLeft(left);
		dsi.setRight(right);

		DisparitySpaceImage<unsigned short> out(100,100,10,20);
		out.set(1);
		DisparitySpaceImage<unsigned short> previous(100+100,1,10,20);

		Aggregate<CensusMatchingCost, unsigned short> aggr = {0,0, out.data(), previous.data(), dsi.data()};
		aggregate_diagonal_upper_all(aggr);

		REQUIRE( out.data()(50,50,12) == 1 );
	}
}

TEST_CASE("Diagonal lower aggregation", "") {
	SECTION("Perform an aggregation") {
		CensusMatchingCost dsi(100,100,10,20,9,7);

		cv::Mat left(100,100, CV_8UC1);
		cv::Mat right(100,100, CV_8UC1);
		left.setTo(cv::Scalar(0));
		right.setTo(cv::Scalar(0));
		dsi.setLeft(left);
		dsi.setRight(right);

		DisparitySpaceImage<unsigned short> out(100,100,10,20);
		out.set(1);
		DisparitySpaceImage<unsigned short> previous(100+100,1,10,20);

		Aggregate<CensusMatchingCost, unsigned short> aggr = {0,0, out.data(), previous.data(), dsi.data()};
		aggregate_diagonal_lower_all(aggr);

		REQUIRE( out.data()(50,50,12) == 1 );
	}
}*/
