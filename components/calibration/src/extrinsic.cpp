#include <loguru.hpp>

#include <ftl/exception.hpp>
#include <ftl/calibration/optimize.hpp>
#include <ftl/calibration/extrinsic.hpp>

#include <fstream>
#include <sstream>

#include <opencv2/calib3d.hpp>

////////////////////////////////////////////////////////////////////////////////

/** check bit i in a */
inline bool hasOne(uint64_t a, unsigned int i) {
	return a & (uint64_t(1) << i);
}

/** all bits set in b are also set in a */
inline bool hasAll(uint64_t a, uint64_t b) {
	return (b & a) == b;
}

/** set bit i in a */
inline void setOne(uint64_t &a, unsigned int i) {
	a |= (uint64_t(1) << i);
}

/** get highest bit*/
inline int hbit(uint64_t a) {
#ifdef __GNUC__
	return 64 - __builtin_clzll(a);
#endif
	int v = 1;
	while (a >>= 1) { v++; }
	return v;
}

inline int popcount(uint64_t bits) {
	#if defined(_MSC_VER)
		return __popcnt64(bits);
	#elif defined(__GNUC__)
		return __builtin_popcountl(bits);
	#else
		int count = 0;
		while (bits != 0) {
			bits = bits >> 1;
			count += uint64_t(1) & bits;
		}
		return count;
	#endif
}

// ==== CalibrationPoints ================================================

namespace ftl {
namespace calibration {

template<typename T>
void CalibrationPoints<T>::addPoints(unsigned int c, const std::vector<cv::Point_<T>>& points) {
	if (hasCamera(c)) {
		throw ftl::exception("Points already set for camera. "
							 "Forgot to call next()?");
	}
	if (current_.object == ~(unsigned int)(0)) {
		throw ftl::exception("Target has be set before adding points.");
	}

	if (objects_[current_.object].size() != points.size()) {
		throw ftl::exception("Number of points must cv::Match object points");
	}

	std::vector<cv::Point_<T>> p(points.begin(), points.end());
	current_.points[c] = p;
	setCamera(c);
};

template<typename T>
unsigned int CalibrationPoints<T>::setObject(const std::vector<cv::Point3_<T>> &object) {
	if (!current_.points.empty()) {
		throw ftl::exception("Points already set, object can not be changed. "
							 "Forgot to call next()?");
	}

	// check if object already exists
	for (unsigned int i = 0; i < objects_.size(); i++) {
		if (objects_[i].size() != object.size()) { continue; }

		bool eq = true;
		for (unsigned int j = 0; j < object.size(); j++) {
			eq &= (objects_[i][j] == object[j]);
		}
		if (eq) {
			current_.object = i;
			return i;
		}
	}

	// not found
	current_.object = objects_.size();
	objects_.push_back(object);
	return current_.object;
}

template<typename T>
unsigned int CalibrationPoints<T>::setObject(const std::vector<cv::Point_<T>> &object) {
	if (!current_.points.empty()) {
		throw ftl::exception("Points already set, object can not be changed. "
							 "Forgot to call next()?");
	}
	std::vector<cv::Point3_<T>> object3d;
	object3d.reserve(object.size());

	for (const auto& p : object) {
		object3d.push_back({p.x, p.y, T(0.0)});
	}
	return setObject(object3d);
}

template<typename T>
void CalibrationPoints<T>::next() {
	if (objects_.empty()) {
		throw ftl::exception("object must be set before calling next()");
	}
	if (current_.cameras == uint64_t(0)) {
		return;
	}

	count_ += objects_[current_.object].size();
	points_.push_back(current_);
	visibility_.update(current_.cameras);
	clear();
}

template<typename T>
void CalibrationPoints<T>::clear() {
	current_ = {uint64_t(0), (unsigned int)(objects_.size()) - 1u, {}, {}};
}

template<typename T>
bool CalibrationPoints<T>::hasCamera(unsigned int c) {
	return hasOne(current_.cameras, c);
}

template<typename T>
void CalibrationPoints<T>::setCamera(unsigned int c) {
	setOne(current_.cameras, c);
}

template<typename T>
int CalibrationPoints<T>::getCount(unsigned int c) {
	return visibility_.count(c);
}

template<typename T>
int CalibrationPoints<T>::getPointsCount() {
	return count_;
}

template<typename T>
const Visibility& CalibrationPoints<T>::visibility() {
	return visibility_;
}

template<typename T>
void CalibrationPoints<T>::setTriangulatedPoints(unsigned int c_base, unsigned int c_match,
	const std::vector<cv::Point3_<T>>& points, int idx) {

	uint64_t required = 0;
	setOne(required, c_base);
	setOne(required, c_match);

	auto itr = points.begin();
	for (unsigned int i = idx; i < points_.size(); i++) {
		if (hasAll(points_[i].cameras, required)) {
			auto obj_sz = objects_[points_[i].object].size();
			std::vector<cv::Point3_<T>> pts;
			pts.reserve(obj_sz);
			for (unsigned int i_obj = 0; i_obj < obj_sz; i_obj++) {
				pts.push_back(*itr);
				itr++;
			}
			points_[i].triangulated[{c_base, c_match}] = pts;
			if (itr == points.end()) { break; }
		}
	}
}

template<typename T>
void CalibrationPoints<T>::resetTriangulatedPoints() {
	for (unsigned int i = 0; i < points_.size(); i++) {
		points_[i].triangulated.clear();
	}
}

template<typename T>
std::vector<std::vector<cv::Point_<T>>> CalibrationPoints<T>::getPoints(const std::vector<unsigned int>& cameras, unsigned int object) {

	std::vector<std::vector<cv::Point_<T>>> points;
	points.resize(cameras.size());
	std::vector<unsigned int> lookup;

	uint64_t required = 0;
	for (unsigned i = 0; i < cameras.size(); i++) {
		setOne(required, cameras[i]);

		if ((cameras[i] + 1) > lookup.size()) {
			lookup.resize(cameras[i] + 1, ~(unsigned int)(0));
		}
		lookup[cameras[i]] = i;
	}

	for (const auto& set : points_) {
		if (!hasAll(set.cameras, required))	{ continue; }
		if (set.object != object)			{ continue; }

		for (auto &[i, data] : set.points) {
			if (!hasOne(required, i)) { continue; }

			points[lookup[i]].insert
				(points[lookup[i]].end(), data.begin(), data.end());
		}
	}

	return points;
}


template<typename T>
std::vector<cv::Point3_<T>> CalibrationPoints<T>::getObject(unsigned int object) {
	return objects_[object];
}

template class CalibrationPoints<float>;
template class CalibrationPoints<double>;

////////////////////////////////////////////////////////////////////////////////

int recoverPose(const cv::Mat &E, const std::vector<cv::Point2d> &_points1,
	const std::vector<cv::Point2d> &_points2, const cv::Mat &_cameraMatrix1,
	const cv::Mat &_cameraMatrix2, cv::Mat &_R, cv::Mat &_t, double distanceThresh,
	cv::Mat &triangulatedPoints) {

	cv::Mat cameraMatrix1;
	cv::Mat cameraMatrix2;
	cv::Mat cameraMatrix;

	cv::Mat points1(_points1.size(), 2, CV_64FC1);
	cv::Mat points2(_points2.size(), 2, CV_64FC1);

	CHECK_EQ(points1.size(), points2.size());

	for (size_t i = 0; i < _points1.size(); i++) {
		auto p1 = points1.ptr<double>(i);
		p1[0] = _points1[i].x;
		p1[1] = _points1[i].y;

		auto p2 = points2.ptr<double>(i);
		p2[0] = _points2[i].x;
		p2[1] = _points2[i].y;
	}

	_cameraMatrix1.convertTo(cameraMatrix1, CV_64F);
	_cameraMatrix2.convertTo(cameraMatrix2, CV_64F);
	cameraMatrix = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);

	double fx1 = cameraMatrix1.at<double>(0,0);
	double fy1 = cameraMatrix1.at<double>(1,1);
	double cx1 = cameraMatrix1.at<double>(0,2);
	double cy1 = cameraMatrix1.at<double>(1,2);

	double fx2 = cameraMatrix2.at<double>(0,0);
	double fy2 = cameraMatrix2.at<double>(1,1);
	double cx2 = cameraMatrix2.at<double>(0,2);
	double cy2 = cameraMatrix2.at<double>(1,2);

	points1.col(0) = (points1.col(0) - cx1) / fx1;
	points1.col(1) = (points1.col(1) - cy1) / fy1;

	points2.col(0) = (points2.col(0) - cx2) / fx2;
	points2.col(1) = (points2.col(1) - cy2) / fy2;

	// TODO mask
	// cameraMatrix = I (for details, see OpenCV's recoverPose() source code)
	// modules/calib3d/src/five-point.cpp (461)
	//
	// https://github.com/opencv/opencv/blob/371bba8f54560b374fbcd47e7e02f015ac4969ad/modules/calib3d/src/five-point.cpp#L461

	return cv::recoverPose( E, points1, points2, cameraMatrix, _R, _t,
							distanceThresh, cv::noArray(), triangulatedPoints);
}

double calibratePair(const cv::Mat &K1, const cv::Mat &D1,
		const cv::Mat &K2, const cv::Mat &D2,
		const std::vector<cv::Point2d> &points1,
		const std::vector<cv::Point2d> &points2,
		const std::vector<cv::Point3d> &object_points, cv::Mat &R,
		cv::Mat &t, std::vector<cv::Point3d> &points_out, bool optimize) {

	cv::Mat F = cv::findFundamentalMat(points1, points2, cv::noArray(), cv::FM_8POINT);
	cv::Mat E = K2.t() * F * K1;

	cv::Mat points3dh;
	// distanceThresh unit?
	recoverPose(E, points1, points2, K1, K2, R, t, 1000.0, points3dh);

	points_out.clear();
	points_out.reserve(points3dh.cols);

	// convert from homogenous coordinates
	for (int col = 0; col < points3dh.cols; col++) {
		CHECK_NE(points3dh.at<double>(3, col), 0);
		cv::Point3d p = cv::Point3d(points3dh.at<double>(0, col),
							points3dh.at<double>(1, col),
							points3dh.at<double>(2, col))
							/ points3dh.at<double>(3, col);
		points_out.push_back(p);
	}

	double s = ftl::calibration::optimizeScale(object_points, points_out);
	t = t * s;

	auto params1 = Camera(K1, D1, cv::Mat::eye(3, 3, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1), {0, 0});
	auto params2 = Camera(K2, D2, R, t, {0, 0});

	auto ba = BundleAdjustment();
	ba.addCamera(params1);
	ba.addCamera(params2);

	for (size_t i = 0; i < points_out.size(); i++) {
		ba.addPoint({points1[i], points2[i]}, points_out[i]);
	}

	// needs to be implemented correctly: optimize each pose of the target
	//ba.addObject(object_points);

	double error = ba.reprojectionError();

	if (optimize) {
		BundleAdjustment::Options options;
		options.optimize_intrinsic = false;
		// any difference if both transformations multiplied with (T_1)^-1
		// (inverse of first camera's transforma)after optimization instead?
		options.fix_camera_extrinsic = {0};
		ba.run(options);
		error = ba.reprojectionError();
	}
	CHECK_EQ(cv::countNonZero(params1.rvec()), 0);
	CHECK_EQ(cv::countNonZero(params1.tvec()), 0);

	return sqrt(error);
}

// ==== Extrinsic Calibration ==================================================

unsigned int ExtrinsicCalibration::addCamera(const CalibrationData::Intrinsic &c) {
	unsigned int idx = calib_.size();
	calib_.push_back({c, {}});
	calib_optimized_.push_back(calib_.back());
	is_calibrated_.push_back(false);
	return idx;
}

unsigned int ExtrinsicCalibration::addCamera(const CalibrationData::Calibration &c) {
	unsigned int idx = calib_.size();
	calib_.push_back(c);
	calib_optimized_.push_back(calib_.back());
	is_calibrated_.push_back(true);
	return idx;
}

unsigned int ExtrinsicCalibration::addStereoCamera(const CalibrationData::Intrinsic &c1, const CalibrationData::Intrinsic &c2) {
	unsigned int idx = calib_.size();
	calib_.push_back({c1, {}});
	calib_optimized_.push_back(calib_.back());
	calib_.push_back({c2, {}});
	calib_optimized_.push_back(calib_.back());
	is_calibrated_.push_back(false);
	is_calibrated_.push_back(false);
	mask_.insert({idx, idx + 1});
	return idx;
}

unsigned int ExtrinsicCalibration::addStereoCamera(const CalibrationData::Calibration &c1, const CalibrationData::Calibration &c2) {
	unsigned int idx = calib_.size();
	calib_.push_back({c1.intrinsic, c1.extrinsic});
	calib_optimized_.push_back(calib_.back());
	calib_.push_back({c2.intrinsic, c2.extrinsic});
	calib_optimized_.push_back(calib_.back());
	is_calibrated_.push_back(c1.extrinsic.valid());
	is_calibrated_.push_back(c2.extrinsic.valid());
	mask_.insert({idx, idx + 1});
	return idx;
}

std::string ExtrinsicCalibration::status() {
	auto str = std::atomic_load(&status_);
	if (str) { return *str; }
	else { return ""; }
}

void ExtrinsicCalibration::updateStatus_(std::string str) {
	std::atomic_store(&status_, std::make_shared<std::string>(str));
}

void ExtrinsicCalibration::calculatePairPoses() {

	const auto& visibility =  points_.visibility();
	// Calibrate all pairs. TODO: might be expensive if number of cameras is high
	// if not performed for all pairs, remaining non-triangulated poits have to
	// be separately triangulated later.

	int i = 1;
	int i_max = (camerasCount() * camerasCount()) / 2 + 1;

	for (unsigned int c1 = 0; c1 < camerasCount(); c1++) {
	for (unsigned int c2 = c1; c2 < camerasCount(); c2++) {

		updateStatus_(	"Calculating pose for pair " +
						std::to_string(i++) + " of " + std::to_string(i_max));

		if (c1 == c2) {
			pairs_[{c1, c2}] = { cv::Mat::eye(cv::Size(3, 3), CV_64FC1),
								 cv::Mat(cv::Size(1, 3), CV_64FC1, cv::Scalar(0.0)),
								 0.0};

			continue;
		}

		if (mask_.count({c1, c2}) > 0 ) { continue; }
		if (pairs_.find({c1, c2}) != pairs_.end()) { continue; }

		// require minimum number of visible points
		if (visibility.count(c1, c2) < min_points_) {
			LOG(WARNING) << "skipped pair (" << c1 << ", " << c2 << "), not enough points";
			continue;
		}

		// calculate paramters and update triangulation

		cv::Mat K1 = calib_[c1].intrinsic.matrix();
		cv::Mat distCoeffs1 = calib_[c1].intrinsic.distCoeffs.Mat();
		cv::Mat K2 = calib_[c2].intrinsic.matrix();
		cv::Mat distCoeffs2 = calib_[c2].intrinsic.distCoeffs.Mat();
		auto pts = points().getPoints({c1, c2}, 0);
		auto object = points().getObject(0);
		cv::Mat R, t;
		std::vector<cv::Point3d> points3d;
		auto rmse = calibratePair(K1, distCoeffs1, K2, distCoeffs2,
			pts[0], pts[1], object, R, t, points3d, true);

		// debug info
		LOG(INFO) << "RMSE (cameras " << c1 << " & " << c2 << "): " << rmse;

		points().setTriangulatedPoints(c1, c2, points3d);

		pairs_[{c1, c2}] = {R, t, rmse};

		cv::Mat R_i, t_i;
		R.copyTo(R_i);
		t.copyTo(t_i);
		transform::inverse(R_i, t_i);
		pairs_[{c2, c1}] = {R_i, t_i, rmse};
	}}
}

void ExtrinsicCalibration::calculateInitialPoses() {
	updateStatus_("Initial poses from chained transformations");

	// mask stereo cameras (do not pairwise calibrate a stereo pair; unreliable)
	auto visibility =  points_.visibility();
	for (const auto& m: mask_) {
		visibility.mask(m.first, m.second);
	}

	// mask cameras which did not have enough points TODO: triangulation later
	// would still be useful (calculate initial poses, then triangulate)
	for (unsigned int c1 = 0; c1 < camerasCount(); c1++) {
	for (unsigned int c2 = c1; c2 < camerasCount(); c2++) {
		if (pairs_.count({c1, c2}) == 0) {
			visibility.mask(c1, c2);
		}

		// mask bad pairs (high rmse)
		/*if (std::get<2>(pairs_.at({c1, c2})) > 16.0) {
			visibility.mask(c1, c2);
		}*/
	}}

	// pick optimal camera: most views of calibration pattern
	c_ref_ = visibility.argmax();

	auto paths = visibility.shortestPath(c_ref_);

	for (unsigned int c = 0; c < camerasCount(); c++) {
		if (c == c_ref_) { continue; }

		cv::Mat R_chain = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
		cv::Mat t_chain = cv::Mat(cv::Size(1, 3), CV_64FC1, cv::Scalar(0.0));

		auto path = paths.to(c);
		do {
			// iterate in reverse order
			auto prev = path.back();
			path.pop_back();
			auto next = path.back();

			cv::Mat R = std::get<0>(pairs_.at({prev, next}));
			cv::Mat t = std::get<1>(pairs_.at({prev, next}));

			CHECK_EQ(R.size(), cv::Size(3, 3));
			CHECK_EQ(t.total(), 3);

			R_chain = R * R_chain;
			t_chain = t + R * t_chain;
		}
		while(path.size() > 1);

		// note: direction of chain in the loop (ref to target transformation)

		calib_[c].extrinsic =
			CalibrationData::Extrinsic(R_chain, t_chain).inverse();
	}
}

static std::vector<bool> visibility(unsigned int ncameras, uint64_t visible) {
	std::vector<bool> res(ncameras, false);
	for (unsigned int i = 0; i < ncameras; i++) {
		res[i] = visible & (uint64_t(1) << i);
	}
	return res;
}

/* absolute difference between min and max for each set of coordinates */
static cv::Point3d absdiff(const std::vector<double> &xs, const std::vector<double> &ys, const std::vector<double> &zs) {
	double minx = INFINITY;
	double maxx = -INFINITY;
	for (auto x : xs) {
		minx = std::min(minx, x);
		maxx = std::max(maxx, x);
	}
	double miny = INFINITY;
	double maxy = -INFINITY;
	for (auto y : ys) {
		miny = std::min(miny, y);
		maxy = std::max(maxy, y);
	}
	double minz = INFINITY;
	double maxz = -INFINITY;
	for (auto z : zs) {
		minz = std::min(minz, z);
		maxz = std::max(maxz, z);
	}
	return {abs(minx - maxx), abs(miny - maxy), abs(minz - maxz)};
}

double ExtrinsicCalibration::optimize() {

	BundleAdjustment ba;
	std::vector<Camera> cameras;
	std::vector<cv::Mat> T; // camera to world

	cameras.reserve(calib_.size());
	unsigned int ncameras = calib_.size();

	for (const auto& c : calib_) {
		auto camera = c;
		T.push_back(c.extrinsic.inverse().matrix());
		cameras.push_back(Camera(camera));
	}
	for (auto &c : cameras) {
		// BundleAdjustment stores pointers; do not resize cameras vector
		ba.addCamera(c);
	}
	// TODO (?) is this good idea?; make optional
	ba.addObject(points_.getObject(0));

	// Transform triangulated points into same coordinates. Poinst which are
	// triangulated multiple times: use median values. Note T[] contains
	// inverse transformations, as points are transformed from camera to world
	// (instead the other way around by parameters in cameras[]).

	updateStatus_("Calculating points in world coordinates");

	// NOTE: above CalibrationPoints datastructure not optimal regarding how
	//		 points are actually used here; BundleAdjustment interface also
	//		 expects flat arrays; overall cv::Mats would probably be better
	//		 suited as they can be easily interpreted as flat arrays or
	//		 multi-dimensional.

	int n_points_bad = 0;
	int n_points_missing = 0;
	int n_points = 0;

	for (const auto& itm : points_.all()) {
		auto sz = points_.getObject(itm.object).size();
		auto vis = visibility(ncameras, itm.cameras);

		for (unsigned int i = 0; i < sz; i++) {
			n_points++;

			// observation and triangulated coordinates; Use {NAN, NAN} for
			// non-visible points (if those are used by mistake, Ceres will
			// fail with error message).
			std::vector<cv::Point2d> obs(ncameras, {NAN, NAN});
			std::vector<double> px;
			std::vector<double> py;
			std::vector<double> pz;

			for (const auto& [c, o] : itm.points) {
				obs[c] = o[i];
			}

			for (const auto [c, pts] : itm.triangulated) {
				auto p = transform::apply(pts[i], T[c.first]);
				px.push_back(p.x);
				py.push_back(p.y);
				pz.push_back(p.z);
			}

			// median coordinate for each axis
			std::sort(px.begin(), px.end());
			std::sort(py.begin(), py.end());
			std::sort(pz.begin(), pz.end());
			cv::Point3d p;

			unsigned int n = px.size();
			unsigned int m = n / 2;
			if (m == 0) {
				n_points_missing++;
				break;
				// not triangulated (see earlier steps)
				// TODO: triangulate here
			}
			if (n % 2 == 0 && n > 1) {
				// mean of two points if number of points even
				cv::Point3d p1 = {px[m - 1], py[m - 1], pz[m - 1]};
				cv::Point3d p2 = {px[m], py[m], pz[m]};
				p = (p1 + p2)/2.0;
			}
			else {
				p = {px[m], py[m], pz[m]};
			}

			// TODO: desgin better check
			if (cv::norm(absdiff(px, py, pz)) > threshold_bad_) {
				n_points_bad++;
				continue;
			}

			ba.addPoint(vis, obs, p);
		}
	}

	if (float(n_points_bad)/float(n_points - n_points_missing) > threhsold_warning_) {
		// print wanrning message; calibration possibly fails if triangulation
		// was very low quality (more than % bad points)
		LOG(ERROR) << "Large variation in "<< n_points_bad << " "
					  "triangulated points. Are initial intrinsic parameters "
					  "good?";
	}

	if (float(n_points_missing)/float(n_points - n_points_bad) > threhsold_warning_) {
		// low number of points; most points only visible in pairs?
		LOG(WARNING) << "Large number of points skipped. Are there enough "
						"visible points between stereo camera pairs? (TODO: "
						"implement necessary triangulation after pair "
						"calibration)";
	}

	updateStatus_("Bundle adjustment");
	options_.fix_camera_extrinsic = {int(c_ref_)};
	options_.verbose = true;
	options_.max_iter = 250; // should converge much earlier

	LOG(INFO) << "fix intrinsics: " << (options_.optimize_intrinsic ? "no" : "yes");
	LOG(INFO) << "fix focal: " << (options_.fix_focal ? "yes" : "no");
	LOG(INFO) << "fix principal point: " << (options_.fix_principal_point ? "yes" : "no");
	LOG(INFO) << "fix distortion: " << (options_.fix_distortion ? "yes" : "no");

	ba.run(options_);
	LOG(INFO) << "removed points: " << ba.removeObservations(2.0);
	ba.run(options_);
	LOG(INFO) << "removed points: " << ba.removeObservations(1.0);
	ba.run(options_);

	calib_optimized_.resize(calib_.size());
	rmse_.resize(calib_.size());

	for (unsigned int i = 0; i < cameras.size(); i++) {
		rmse_[i] = ba.reprojectionError(i);
		auto intr = cameras[i].intrinsic();
		calib_optimized_[i] = calib_[i];
		calib_optimized_[i].intrinsic.set(intr.matrix(), intr.distCoeffs.Mat(), intr.resolution);
		calib_optimized_[i].extrinsic.set(cameras[i].rvec(), cameras[i].tvec());
	}

	rmse_total_ = ba.reprojectionError();

	LOG(INFO) << "reprojection error (all cameras): " << rmse_total_;
	return rmse_total_;
}

double ExtrinsicCalibration::run() {
	updateStatus_("Starting");
	points_.resetTriangulatedPoints();
	pairs_.clear();
	calculatePairPoses();
	calculateInitialPoses();
	return optimize();
}

const CalibrationData::Calibration& ExtrinsicCalibration::calibration(unsigned int c) {
	return calib_.at(c);
}

const CalibrationData::Calibration& ExtrinsicCalibration::calibrationOptimized(unsigned int c) {
	return calib_optimized_.at(c);
}

double ExtrinsicCalibration::reprojectionError(unsigned int c) {
	return rmse_.at(c);
}

double ExtrinsicCalibration::reprojectionError() {
	return rmse_total_;
}

bool ExtrinsicCalibration::toFile(const std::string& fname) {
	points_.clear();
	std::ofstream ofs(fname, std::ios_base::trunc);
	msgpack::pack(ofs, *this);
	ofs.close();
	return true;
}

bool ExtrinsicCalibration::fromFile(const std::string& fname) {

	points_ = CalibrationPoints<double>();
	mask_ = {};
	calib_ = {};

	std::ifstream ifs(fname);
	std::stringstream buf;
	msgpack::object_handle oh;

	buf << ifs.rdbuf();
	msgpack::unpack(oh, buf.str().data(), buf.str().size());
	oh.get().convert(*this);

	return true;
}


}
}
