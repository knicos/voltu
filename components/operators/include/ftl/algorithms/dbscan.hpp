#ifndef HPP_FTL_ALGORITHMS_DBSCAN_
#define HPP_FTL_ALGORITHMS_DBSCAN_

#include <vector>
#include <deque>
#include <opencv2/core/core.hpp>

namespace ftl {

/**
 * DBSCAN clustering algorithm. Iterates over each points and assigns a label
 * based on local neighborhood. Complexity O(n*O(RangeQuery)) for n points.
 *
 * points		Input parameter: points
 * RangeQuery	function vector<size_t>(points, i, radius) which returns point
 * 				indices (excluding input point with index i) which are within
 * 				given radius. Called at least once for each point (but at most
 * 				twice).
 * min_points	DBSCAN parameter: minimum cluster size (core point).
 * radius		DBSCAN parameter: search radius
 * labels		Output paramters: cluster labels. Negative labels are used for
 * 				noise.
 * centroids	Output parameter: cluster centroids
 */
template<typename T>
void dbscan(const std::vector<T> &points,
			std::function<std::vector<size_t>(const std::vector<T>&, size_t, float)> RangeQuery,
			unsigned int min_points, float radius,
			std::vector<short> &labels, std::vector<T> &centroids) {

	const short NONE = -2;
	const short NOISE = -1;

	labels.resize(points.size());
	std::fill(labels.begin(), labels.end(), NONE);

	int cluster_count = 0;

	for (unsigned i = 0; i < points.size(); i++) {
		short cluster = NONE;

		if (labels[i] != NONE) {
			continue;
		}

		// get neighbours of points[i]
		std::vector<size_t> neighbors = RangeQuery(points, i, radius);

		if (neighbors.size() < min_points) {
			labels[i] = NOISE;
			continue;
		}

		// assign new cluster id
		cluster = cluster_count++;

		labels[i] = cluster;
		T centroid = points[i];
		int n_points = 1;

		// seed_set: neighboring points to this cluster
		std::deque<size_t> seed_set;
		for (const auto &n : neighbors) {
			seed_set.push_back(n);
		}

		while(!seed_set.empty()) {
			auto i_n  = seed_set.front();
			seed_set.pop_front();

			if (labels[i_n] == NOISE) {
				// add to cluster (few lines down)
			}
			else if (labels[i_n] != NONE){
				continue;
			}

			labels[i_n] = cluster;
			centroid += points[i_n];
			n_points++;

			neighbors = RangeQuery(points, i_n, radius);

			if (neighbors.size() < min_points) {
				continue;
			}
			else {
				for (const auto &n : neighbors) {
					seed_set.push_back(n);
				}
			}
		}

		centroids.push_back(centroid/n_points);
	}
}

}
#endif
