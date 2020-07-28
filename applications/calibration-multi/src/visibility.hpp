#pragma once

#include <opencv2/core.hpp>

using cv::Mat;
using std::vector;
using std::pair;

class Visibility {
public:
	explicit Visibility(int n_cameras);

	/**
	 * @breif	Update visibility graph.
	 * @param	Which cameras see the feature(s) in this iteration
	 */
	void update(vector<int> &visible);

	/**
	 * @brief	For all cameras, find shortest (optimal) paths to reference
	 * 			camera
	 * @param	Id of reference camera
	 *
	 * Calculates shortest path in weighted graph using Dijstra's
	 * algorithm. Weights are inverse of views between cameras (nodes)
	 *
	 * @todo	Add constant weight for each edge (prefer less edges)
	 */
	vector<vector<pair<int, int>>> findShortestPaths(int reference);

	vector<int> getClosestCameras(int c);
	void deleteEdge(int camera1, int camera2);

	int getOptimalCamera();

	/** @brief Returns the smallest visibility count (any camera)
	 */
	int getMinVisibility();

	/** @brief Returns the visibility camera's value
	 */
	int getViewsCount(int camera);

protected:
	/**
	 * @brief	Find shortest path between nodes
	 * @param	Source node id
	 * @param	Destination node id
	 */
	vector<pair<int, int>> findShortestPath(int from, int to);

private:
	int n_cameras_;		// @brief number of cameras
	cv::Mat visibility_;	// @brief adjacency matrix
	vector<int> count_;
};
