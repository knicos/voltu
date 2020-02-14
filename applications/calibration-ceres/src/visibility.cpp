#include "visibility.hpp"
#include "loguru.hpp"

#include <limits>
#include <algorithm>
#include <queue>

using std::vector;
using std::pair;
using std::make_pair;

using ftl::calibration::Paths;
using ftl::calibration::Visibility;

template<typename T>
Paths<T>::Paths(const vector<int> &previous, const vector<T> &distances) :
	previous_(previous), distances_(distances) {}

template<typename T>
vector<int> Paths<T>::from(int i) const {
	vector<int> path;

	if (previous_[i] == -1) { return {}; }

	int current = i;
	do {
		if (distance(i) == std::numeric_limits<T>::max()) { return {}; } // no path

		path.push_back(current);
		current = previous_[current];
	}
	while (previous_[current] != -1);

	return path;
}

template<typename T>
vector<int> Paths<T>::to(int i) const {
	vector<int> path = from(i);
	std::reverse(path.begin(), path.end());
	return path;
}

template<typename T>
T Paths<T>::distance(int i) const {
	return distances_[i];
}

template<typename T>
bool Paths<T>::connected() const {
	for (const auto &distance : distances_) {
		if (distance == std::numeric_limits<T>::max()) { return false; }
	}
	return true;
}

template<typename T>
std::string Paths<T>::to_string() const {
	std::stringstream sb;

	int node = -1;
	for (size_t i = 0; i < distances_.size(); i++) {
		if (previous_[i] == -1) {
			node = i;
			break;
		}
	}

	for (size_t i = 0; i < distances_.size(); i++) {
		sb << node;

		for (const auto &p : to(i)) {
			sb << " -> " << p;
		}

		sb << " (" << distances_[i] << ")\n";
	}
	return sb.str();
}

template class Paths<int>;
template class Paths<float>;
template class Paths<double>;

////////////////////////////////////////////////////////////////////////////////

template<typename T>
static bool isValidAdjacencyMatrix(const vector<vector<T>> &graph) {
	size_t nrows = graph.size();
	for (const auto &col : graph) {
		if (nrows != col.size()) { return false; }
	}

	for (size_t r = 0; r < nrows; r++) {
		for (size_t c = r; c < nrows; c++) {
			if (graph[r][c] != graph[c][r]) {
				return false; }
		}
	}

	return true;
}

template<typename T>
static vector<int> find_neighbors(int i, const vector<vector<T>> &graph) {
	vector<int> res;

	for (size_t j = 0; j < graph.size(); j++) {
		if (graph[i][j] > 0) { res.push_back(j); }
	}

	return res;
}

template<typename T>
static pair<vector<int>, vector<T>> dijstra_impl(const int i, const vector<vector<T>> &graph) {
	// distance, index
	std::priority_queue<pair<T, int>, vector<pair<T, int>>, std::greater<pair<T, int>>> pq;

	pq.push(make_pair(0, i));

	vector<T> distance_path(graph.size(), std::numeric_limits<T>::max());
	vector<int> previous(graph.size(), -1);

	distance_path[i] = 0;

	while(!pq.empty()) {
		int current = pq.top().second;
		pq.pop();

		for (int n : find_neighbors(current, graph)) {
			T cost = graph[current][n] + distance_path[current];
			if (distance_path[n] > cost) {
				distance_path[n] = cost;
				pq.push(make_pair(cost, n));
				previous[n] = current;
			}
		}
	}

	return make_pair(previous, distance_path);
}

template<typename T>
Paths<T> ftl::calibration::dijstra(const int i, const vector<vector<T>> &graph) {
	auto tmp = dijstra_impl(i, graph);
	return Paths<T>(tmp.first, tmp.second);
}

template Paths<int> ftl::calibration::dijstra(const int i, const vector<vector<int>> &graph);
template Paths<float> ftl::calibration::dijstra(const int i, const vector<vector<float>> &graph);
template Paths<double> ftl::calibration::dijstra(const int i, const vector<vector<double>> &graph);

////////////////////////////////////////////////////////////////////////////////

Visibility::Visibility(int n_cameras) :
	n_cameras_(n_cameras),
	graph_(n_cameras, vector(n_cameras, 0)),
	mask_(n_cameras, vector(n_cameras, false))
	{}

Visibility::Visibility(const vector<vector<int>> &graph) :
	n_cameras_(graph.size()),
	graph_(graph),
	mask_(graph.size(), vector(graph.size(), false)) {

		if (!isValidAdjacencyMatrix(graph)) {
			throw std::exception();
		}
}

void Visibility::init(int n_cameras) {
	n_cameras_ = n_cameras;
	graph_ = vector(n_cameras, vector(n_cameras, 0));
	mask_ = vector(n_cameras, vector(n_cameras, false));
}

template<typename T>
void Visibility::update(const vector<T> &add) {
	if ((int) add.size() != n_cameras_) { throw std::exception(); }

	for (int i = 0; i < n_cameras_; i++) {
		if (!add[i]) { continue; }

		for (int j = 0; j < n_cameras_; j++) {
			if (i == j) { continue; }
			if (add[j]) { graph_[i][j] += 1; }
		}
	}
}

template void Visibility::update(const std::vector<int> &add);
template void Visibility::update(const std::vector<bool> &add);

void Visibility::mask(int a, int b) {
	mask_[a][b] = true;
	mask_[b][a] = true;
}

void Visibility::unmask(int a, int b) {
	mask_[a][b] = false;
	mask_[b][a] = false;
}

int Visibility::distance(int a, int b) const {
	return graph_[a][b];
}

Paths<float> Visibility::shortestPath(int i) const {
	if ((i < 0) || (i >= n_cameras_)) { return Paths<float>({}, {}); /* throw exception */}

	vector<vector<float>> graph(n_cameras_, vector<float>(n_cameras_, 0.0f));
	for (int r = 0; r < n_cameras_; r++) {
		for (int c = 0; c < n_cameras_; c++) {
			float v = graph_[r][c];
			if ((v != 0) && !mask_[r][c]) { graph[r][c] = 1.0f / v; }
		}
	}

	auto res = dijstra_impl(i, graph);
	for (float &distance : res.second) {
		distance = 1.0f / distance;
	}

	return Paths<float>(res.first, res.second);
}
