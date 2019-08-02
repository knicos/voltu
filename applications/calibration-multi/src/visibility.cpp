#include <numeric>
#include <loguru.hpp>
#include <queue>

#include "visibility.hpp"

using cv::Mat;
using cv::Scalar;
using cv::Size;
using std::vector;
using std::pair;
using std::make_pair;

Visibility::Visibility(int n_cameras) : n_cameras_(n_cameras) {
	visibility_ = Mat(Size(n_cameras, n_cameras), CV_32SC1, Scalar(0));
	count_ = vector(n_cameras, 0);
}

void Visibility::update(vector<int> &visible) {
	DCHECK(visible.size() == (size_t) n_cameras_);

	for (int i = 0; i < n_cameras_; i++) {
		if (visible[i] == 0) continue;
		count_[i]++;

		for (int j = 0; j < n_cameras_; j++) {
			if (i == j) continue;
			if (visible[j] == 1) visibility_.at<int>(i, j)++;
		}
	}
}

int Visibility::getOptimalCamera() {
	// most visible on average
	int best_i;
	double best_score = -INFINITY;
	for (int i = 0; i < visibility_.rows; i++) {
		double score = 0.0;
		for (int x = 0; x < visibility_.cols; x++) {
			score += visibility_.at<int>(i, x);
		}
		score = score / (double) visibility_.cols;
		if (score > best_score) {
			best_i = i;
			best_score = score;
		}
	}
	
	return best_i;
}

int Visibility::getMinVisibility() {
	int min_i;
	int min_count = INT_MAX;

	for (int i = 0; i < n_cameras_; i++) {
		if (count_[i] < min_count) {
			min_i = i;
			min_count = count_[i];
		}
	}
	
	return min_count;
}

int Visibility::getViewsCount(int camera) {
	return count_[camera];
}

vector<vector<pair<int, int>>> Visibility::findShortestPaths(int reference) {
	DCHECK(reference < n_cameras_);

	vector<vector<pair<int, int>>> res(n_cameras_);
	for (int i = 0; i < n_cameras_; i++) {
		res[i] = findShortestPath(i, reference);
	}
	
	return res;
}

vector<pair<int, int>> Visibility::findShortestPath(int from, int to) {
	if (from == to) return vector<pair<int, int>>();

	vector<bool> visited(n_cameras_, false);
	vector<double> distances(n_cameras_, INFINITY);
	vector<int> previous(n_cameras_, -1);
	
	distances[from] = 0.0;

	auto cmp = [](pair<int, double> u, pair<int, double> v) { return u.second > v.second; };
	std::priority_queue<pair<int, double>, vector<pair<int, double>>, decltype(cmp)> pq(cmp);

	pq.push(make_pair(from, distances[from]));

	while(!pq.empty()) {
		pair<int, double> current = pq.top();
		pq.pop();

		int current_id = current.first;
		double current_distance = distances[current_id];

		visited[current_id] = true;

		for (int i = 0; i < n_cameras_; i++) {
			int count = visibility_.at<int>(current_id, i);
			if (count == 0) continue; // not connected

			double distance = 1.0 / (double) count;
			double new_distance = current_distance + distance;
			
			if (distances[i] > new_distance) {
				distances[i] = new_distance;
				previous[i] = current_id;

				pq.push(make_pair(i, distances[i]));
			}
		}
	}

	vector<pair<int, int>> res;
	int prev = previous[to];
	int current = to;

	do {
		res.push_back(make_pair(current, prev));
		current = prev;
		prev = previous[prev];
	}
	while(prev != -1);

	std::reverse(res.begin(), res.end());
	return res;
}

vector<int> Visibility::getClosestCameras(int c) {

	// initialize original index locations
	vector<int> idx(n_cameras_);
	iota(idx.begin(), idx.end(), 0);
	int* views = visibility_.ptr<int>(c);
	
	// sort indexes based on comparing values in v
	sort(idx.begin(), idx.end(),
		[views](size_t i1, size_t i2) {return views[i1] < views[i2];});

	return idx;
}