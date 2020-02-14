#pragma once
#ifndef _FTL_VISIBILITY_HPP_
#define _FTL_VISIBILITY_HPP_

#include <vector>
#include <string>

namespace ftl {
namespace calibration {

/**
 * @brief Result from Dijkstra's algorithm.
 */
template<typename T>
class Paths {
public:
	Paths(const std::vector<int> &previous, const std::vector<T> &distances);

	/**
	 * @brief Shortest path from node i. Same as to(i) in reverse order
	 */
	std::vector<int> from(int i) const;

	/**
	 * @brief Shortest to node i. Same as from(i) in reverse order.
	 */
	std::vector<int> to(int i) const;

	/**
	 * @brief Is graph connected, i.e. all nodes are reachable.
	 */
	bool connected() const;

	/**
	 * @brief Distance to node i
	 */
	T distance(int i) const;

	std::string to_string() const;

private:
	std::vector<int> previous_;
	std::vector<T> distances_;
};

/**
 * @brief Dijstra's algorithm: shortest paths from node i.
 * @param i		node index
 * @param graph	adjacency matrix
 *
 * dijstra<int>(), dijstra<float>() and dijstra<double>() defined.
 */
template<typename T>
Paths<T> dijstra(const int i, const std::vector<std::vector<T>> &graph_);

class Visibility {
	public:
	Visibility() {};
	explicit Visibility(int n_cameras);
	explicit Visibility(const std::vector<std::vector<int>> &graph);

	void init(int n_cameras);

	template<typename T>
	void update(const std::vector<T> &add);

	void mask(int a, int b);
	void unmask(int a, int b);

	/**
	 * @brief Find most visibility shortest path to camera i
	 */
	Paths<float> shortestPath(int i) const;

	protected:
	std::vector<int> neighbors(int i) const;
	int distance(int a, int b) const;

	private:

	int n_cameras_;
	// adjacency matrix
	std::vector<std::vector<int>> graph_;
	// masked values (mask_[i][j]) are not used
	std::vector<std::vector<bool>> mask_;
};

}
}

#endif
