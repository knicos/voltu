#include <catch.hpp>
#include <random>


#include <opencv2/core.hpp>
#include <ftl/algorithms/dbscan.hpp>

#include <ftl/profiler.hpp>

#include "data.hpp"

using std::vector;

template<typename T>
static vector<size_t> linearSearch(const vector<T> &points, size_t idx, float radius) {
	vector<size_t> neighbors;
	for (auto i = 0u; i < points.size(); i++) {
		if (i == idx) {
			continue;
		}
		if (cv::norm(points[idx] - points[i]) < radius) {
			neighbors.push_back(i);
		}
	}
	return neighbors;
}

TEST_CASE("DBSCAN 3D clustering (linear search)") {
	vector<cv::Vec3f> points {
		{1.0,2.1,3.0},
		{1.0,1.9,3.0},
		{1.0,2.0,3.0},
		{1.0,1.9,3.0},
		{1.0,2.1,3.0},

		{3.0,2.1,1.0},
		{3.0,2.0,1.0},
		{3.0,2.0,1.0},
		{3.0,2.0,1.0},
		{3.0,1.9,1.0}
	};

	vector<cv::Vec3f> centroids;
	vector<short> labels;
	ftl::dbscan<cv::Vec3f>(points, linearSearch<cv::Vec3f>, 3, 1.0f, labels, centroids);

	REQUIRE(centroids.size() == 2);
	REQUIRE(centroids[0] == cv::Vec3f(1,2,3));
	REQUIRE(centroids[1] == cv::Vec3f(3,2,1));
}

TEST_CASE("DBSCAN 3D clustering (random points)") {

	std::random_device rd;
	std::mt19937::result_type seed = rd();

	vector<cv::Vec3f> true_centroids = {
		{ 1, 5, 3},
		{ 3, 5, 1},
		{ 0, 0, 0},
		{ 3, 3, 3},
		{-3,-3,-3},
		{ 7, 7, 7},
		{-7,-7,-7},
	};

	int n_points = 16;
	float sigma = 0.33;
	float eps = sigma; // error threshold for test case

	vector<cv::Vec3f> points;
	std::mt19937 gen(seed);

	for (const auto &c : true_centroids) {
		std::normal_distribution<float> x{c[0], sigma};
		std::normal_distribution<float> y{c[1], sigma};
		std::normal_distribution<float> z{c[2], sigma};

		for (int i = 0; i < n_points; i++) {
			points.push_back({x(gen), y(gen), z(gen)});
		}
	}

	vector<cv::Vec3f> centroids;
	vector<short> labels;
	ftl::dbscan<cv::Vec3f>(points, linearSearch<cv::Vec3f>, 8, 1.0f, labels, centroids);

	REQUIRE(centroids.size() == true_centroids.size());
	for (unsigned i = 0; i < true_centroids.size(); i++) {
		// assumes same order as points were added (no shuffle)
		REQUIRE(cv::norm(centroids[i] - true_centroids[i]) < eps);
	}
}

TEST_CASE("DBSCAN 2D clustering (noisy moons)") {

	vector<cv::Vec2f> centroids;
	vector<short> labels;
	{
		//ftl::Profiler __profile(__func__, "DBSCAN 1500 points linear search", 0);
		//__profile.verbosity(1);

		// ~ 10ms (release)
		ftl::dbscan<cv::Vec2f>(noisy_moons, linearSearch<cv::Vec2f>, 5, 0.2f, labels, centroids);
	}

	// assumes clustering returns same labels each time
	REQUIRE(centroids.size() == 2);
	REQUIRE(cv::norm(centroids[0] - cv::Vec2f(1.0, 0.0)) < 0.15); // 0.13359162681252454
	REQUIRE(cv::norm(centroids[1] - cv::Vec2f(0.0, 0.5)) < 0.15); // 0.13651460122147505

	for (unsigned i = 0; i < labels.size(); i++) {
		if (labels[i] < 0) continue; // label: NOISE
		REQUIRE(labels[i] == noisy_moons_labels[i]);
	}
}

TEST_CASE("DBSCAN 2D clustering (noisy circles)") {

	vector<cv::Vec2f> centroids;
	vector<short> labels;
	{
		//ftl::Profiler __profile(__func__, "DBSCAN 1500 points linear search", 0);
		//__profile.verbosity(1);

		// ~10ms (release)
		ftl::dbscan<cv::Vec2f>(noisy_circles, linearSearch<cv::Vec2f>, 5, 0.1f, labels, centroids);
	}

	// assumes clustering returns same labels each time
	REQUIRE(centroids.size() == 2);
	REQUIRE(cv::norm(centroids[0]) < 0.01); // 0.0008899436718976423
	REQUIRE(cv::norm(centroids[0]) < 0.01); // 0.0014477936451883612
	for (unsigned i = 0; i < labels.size(); i++) {
		if (labels[i] < 0) continue; // label: NOISE
		REQUIRE(labels[i] == noisy_circles_labels[i]);
	}
}

