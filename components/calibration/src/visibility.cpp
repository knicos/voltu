#include <loguru.hpp>

#include <numeric>
#include <limits>
#include <algorithm>
#include <queue>

#include <opencv2/core.hpp>

#include <ftl/exception.hpp>
#include <ftl/calibration/visibility.hpp>

using cv::Mat;
using cv::Scalar;
using cv::Size;
using std::vector;
using std::pair;
using std::make_pair;

using std::vector;
using std::pair;
using std::make_pair;

using ftl::calibration::Paths;
using ftl::calibration::Visibility;

/** get highest bit*/
inline int hbit(uint64_t a) {
#ifdef __GNUC__
	return 64 - __builtin_clzll(a);
#endif
	int v = 1;
	while (a >>= 1) { v++; }
	return v;
}

template<typename T>
Paths<T>::Paths(int id, const vector<int> &previous, const vector<T> &distances) :
	id_(id), previous_(previous), distances_(distances) {}

template<typename T>
vector<int> Paths<T>::from(int i) const {
	vector<int> path;
	path.push_back(i);

	if (previous_[i] == -1) { return path; }
	int current = previous_[i];

	while (previous_[current] != -1) {
		if (distance(i) == std::numeric_limits<T>::max()) { return {}; } // no path

		path.push_back(current);
		current = previous_[current];
	}

	path.push_back(id_);
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
	return Paths<T>(i, tmp.first, tmp.second);
}

template Paths<int> ftl::calibration::dijstra(const int i, const vector<vector<int>> &graph);
template Paths<float> ftl::calibration::dijstra(const int i, const vector<vector<float>> &graph);
template Paths<double> ftl::calibration::dijstra(const int i, const vector<vector<double>> &graph);

////////////////////////////////////////////////////////////////////////////////

Visibility::Visibility(int n_cameras) :
	n_cameras_(n_cameras),
	n_max_(0),
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
	if ((int) add.size() != n_cameras_) {
		throw ftl::exception("number of points must match number of cameras");
	}
	n_max_ = n_cameras_;

	for (int i = 0; i < n_cameras_; i++) {
		if (!add[i]) { continue; }

		for (int j = 0; j < n_cameras_; j++) {
			if (add[j]) { graph_[i][j] += 1; }
		}
	}
}

void Visibility::update(uint64_t add) {
	if (n_cameras_ > 64) {
		throw ftl::exception("Bitmask update only if number of cameras less than 64");
	}
	n_max_ = std::max(n_max_, hbit(add));

	for (int i = 0; i < n_max_; i++) {
		if (!(add & (uint64_t(1) << i))) { continue; }

		for (int j = 0; j < n_max_; j++) {
			if (add & (uint64_t(1) << j)) {
				graph_[i][j] += 1;
			}
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

int Visibility::count(int camera) const {
	return graph_[camera][camera];
}

int Visibility::count(int camera1, int camera2) const {
	return graph_[camera1][camera2];
}

float Visibility::distance(int a, int b) const {
	int v = graph_[a][b];
	if (v == 0) { return 0.0f; }
	return 1.0f/float(v);
}

Paths<float> Visibility::shortestPath(int i) const {
	if ((i < 0) || (i >= n_max_)) { throw ftl::exception("Invalid index"); }

	vector<vector<float>> graph(n_max_, vector<float>(n_max_, 0.0f));
	for (int r = 0; r < n_max_; r++) {
		for (int c = 0; c < n_max_; c++) {
			if (r == c) { continue; }

			if (!mask_[r][c]) {
				// use inverse of count as distance in graph
				graph[r][c] = distance(r, c);
			}
		}
	}

	auto res = dijstra_impl(i, graph);
	for (float &distance : res.second) {
		distance = 1.0f / distance;
	}

	return Paths<float>(i, res.first, res.second);
}

int Visibility::argmax() const {
	int a = -1;
	int v = std::numeric_limits<int>::min();
	for (int i = 0; i < n_max_; i++) {
		if (count(i) > v) {
			v = count(i);
			a = i;
		}
	}
	return a;
}

int Visibility::argmin() const {
	int a = -1;
	int v = std::numeric_limits<int>::max();
	for (int i = 0; i < n_max_; i++) {
		if (count(i) < v) {
			v = count(i);
			a = i;
		}
	}
	return a;
}
