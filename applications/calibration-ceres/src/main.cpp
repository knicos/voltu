#include "loguru.hpp"

#include <tuple>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "optimization.hpp"
#include "visibility.hpp"
#include "calibration.hpp"

using std::string;
using std::vector;
using std::map;
using std::reference_wrapper;

using std::tuple;
using std::pair;
using std::make_pair;

using cv::Mat;
using cv::Size;

using cv::Point2d;
using cv::Point3d;

using namespace ftl::calibration;

void loadData(	const string &fname,
				Visibility &visibility,
				CalibrationData &data) {

	vector<Mat> K;
	vector<vector<int>> visible;
	vector<vector<Point2d>> points;

	cv::FileStorage fs(fname, cv::FileStorage::READ);

	fs["K"] >> K;
	fs["points2d"] >> points;
	fs["visible"] >> visible;
	fs.release();

	int ncameras = K.size();
	int npoints = points[0].size();

	visibility.init(ncameras);
	data.init(ncameras);

	for (int i = 0; i < npoints; i+= 1) {

		vector<bool> v(ncameras, 0);
		vector<Point2d> p(ncameras);

		for (int k = 0; k < ncameras; k++) {
			v[k] = visible[k][i];
			p[k] = points[k][i];
		}

		visibility.update(v);
		data.addObservation(v, p);
	}

	for (size_t i = 1; i < K.size(); i += 2) {
		// mask right cameras
		visibility.mask(i-1, i);
	}

	for (int i = 0; i < ncameras; i++) {
		data.getCamera(i).setIntrinsic(K[i]);
	}
}

void calibrate(const string &fname) {

	Visibility visibility;
	CalibrationData data;

	loadData(fname, visibility, data);

	// 2x 15cm squares (ArUco tags) 10cm apart
	vector<Point3d> object_points = {
			Point3d(0.0, 0.0, 0.0),
			Point3d(0.15, 0.0, 0.0),
			Point3d(0.15, 0.15, 0.0),
			Point3d(0.0, 0.15, 0.0),

			Point3d(0.25, 0.0, 0.0),
			Point3d(0.40, 0.0, 0.0),
			Point3d(0.40, 0.15, 0.0),
			Point3d(0.25, 0.15, 0.0)
	};

	int refcamera = 0;
	auto path = visibility.shortestPath(refcamera);

	map<pair<int, int>, pair<Mat, Mat>> transformations;

	// Needs better solution which allows simple access to all estimations.
	// Required for calculating average coordinates and to know which points
	// are missing.

	vector<tuple<int, vector<Point3d>>> points_all;

	transformations[make_pair(refcamera, refcamera)] =
		make_pair(Mat::eye(3, 3, CV_64FC1), Mat::zeros(3, 1, CV_64FC1));

	for (int i = 0; i < data.ncameras(); i++) {

		// Calculate initial translation T from refcam. Visibility graph is
		// used to create a chain of transformations from refcam to i.

		int current =  refcamera;
		if (i == refcamera) { continue; }

		Mat D = Mat::zeros(1, 5, CV_64FC1);
		Mat R = Mat::eye(3, 3, CV_64FC1);
		Mat t = Mat::zeros(3, 1, CV_64FC1);

		for (const int &to : path.to(i)) {
			auto edge = make_pair(current, to);

			if (transformations.find(edge) == transformations.end()) {
				Mat R_;
				Mat t_;
				vector<Point3d> points;

				auto observations = data.getObservations({current, to});
				double err = computeExtrinsicParameters(
					data.getCamera(current).intrinsicMatrix(),
					data.getCamera(current).distortionCoefficients(),
					data.getCamera(to).intrinsicMatrix(),
					data.getCamera(to).distortionCoefficients(),
					observations[0], observations[1],
					object_points, R_, t_, points);

				LOG(INFO) << current << " -> " << to << " (RMS error: " << err << ")";

				transformations[edge] = make_pair(R_, t_);
				points_all.push_back(make_tuple(current, points));
			}

			const auto [R_update, t_update] = transformations[edge];

			R = R * R_update;
			t = R_update * t + t_update;

			current = to;
		}

		transformations[make_pair(refcamera, i)] = make_pair(R.clone(), t.clone());

		data.getCamera(i).setExtrinsic(R, t);
	}

	// TODO: see points_all comment
	/*
	for (auto& [i, points] : points_all) {

		Mat R = data.getCamera(i).rmat();
		Mat t = data.getCamera(i).tvec();
		transform::inverse(R, t); // R, t: refcam -> i

		CHECK(points.size() == points_ref.size());

		for (size_t j = 0; j < points.size(); j++) {
			Point3d &point = points_ref[j];

			if (point != Point3d(0,0,0)) {
				point = (transform::apply(points[j], R, t) + point) / 2.0;
			}
			else {
				point = transform::apply(points[j], R, t);
			}
		}
	}
	*/
	vector<int> idx;
	BundleAdjustment ba;

	ba.addCameras(data.getCameras());

	vector<bool> visible(data.ncameras());
	vector<Point2d> observations(data.ncameras());

	int missing = 0;
	for (int i = 0; i < data.npoints(); i++) {
		for (int j = 0; j < data.ncameras(); j++) {
			visible[j] = data.isVisible(j, i);
			observations[j] = data.getObservation(j, i);
		}

		// TODO: see points_all comment
		if (data.getPoint(i) == Point3d(0.,0.,0.)) {
			missing++;
			continue;
		}

		ba.addPoint(visible, observations, data.getPoint(i));
	}

	LOG(INFO) << "missing points: " << missing;
	LOG(INFO) << "Initial reprojection error: " << ba.reprojectionError();

	BundleAdjustment::Options options;
	options.verbose = false;
	options.optimize_intrinsic = true;

	ba.run(options);

	LOG(INFO) << "Finale reprojection error: " << ba.reprojectionError();

	// calibration done, updated values in data
}

int main(int argc, char* argv[]) {
	return 0;
}
