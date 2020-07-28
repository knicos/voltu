#pragma once

#include <ftl/utility/msgpack.hpp>

#include <ftl/calibration/visibility.hpp>
#include <ftl/calibration/structures.hpp>
#include <ftl/calibration/optimize.hpp>
#include <opencv2/core.hpp>

#include <ftl/utility/msgpack.hpp>

#include <set>

namespace ftl {
namespace calibration {

/**
 * Helper for saving data from multiple cameras and sets image points. Each
 * set of images dosn't have to be complete (all cameras included).
 *
 * Implementation limit: maximum number of cameras limited to 64; Valid camera
 * indices between 0 and 63; other values are UB!
 *
 * Template parameter float or double.
 */
template<typename T>
class CalibrationPoints {
public:
	struct Points {
		// camera index
		uint64_t cameras;
		 // object index
		unsigned int object;
		// points in image coordinates, camera index as key
		std::map<unsigned int, std::vector<cv::Point_<T>>> points;
		// triangulated points, camera pair as map key
		std::map<std::pair<unsigned int, unsigned int>,
				 std::vector<cv::Point3_<T>>> triangulated;

		bool has(unsigned int c) const { return (cameras & (uint64_t(1) << c)); }

		MSGPACK_DEFINE(cameras, object, points, triangulated);
	};

	CalibrationPoints() : count_(0), visibility_(64), current_{0, ~(unsigned int)(0), {}, {}} {};

	/** Set calibration target. Can be changed after calling and before adding
	 * any points next(). */

	/* 2d (planar) target. Returns object ID */
	unsigned int setObject(const std::vector<cv::Point_<T>> &target);
	/* 3d target. Returns object ID */
	unsigned int setObject(const std::vector<cv::Point3_<T>> &target);

	/* Add points for current set. Points can only be set once for each set. */
	void addPoints(unsigned int c, const std::vector<cv::Point_<T>>& points);

	/** Continue next set of images. Target must be set. If no points were added
	 * next() is no-op. */
	void next();

	/** Set triangulated points. Note: flat input.
	 * @param	c_base	base camera (origin to point coordinates)
	 * @param	c_match	match camera
	 * @param	points	points
	 * @param	idx		index offset, if more image points are added, adjust idx
	 * 					accordingly (value of getPointsCount() before adding new
	 * 					points).
	 */
	void setTriangulatedPoints(unsigned int c_base, unsigned int c_match, const std::vector<cv::Point3_<T>>& points, int idx=0);
	void resetTriangulatedPoints();
	/** TODO: same as above but non-flat input
	void setTriangulatedPoints(unsigned int c_base, unsigned int c_match, const std::vector<std::vector<cv::Point3_<T>>>& points, int idx=0);
	*/

	/** Clear current set of points (clears queue for next()) */
	void clear();

	/** Get count (how many sets) for camera(s). */
	int getCount(unsigned int c);
	int getCount(std::vector<unsigned int> cs);

	/** total number of points */
	int getPointsCount();

	/** Get intersection of points for given cameras. Returns vector of Points
	 * contain object and vector of image points. Image points in same order as
	 * in input parameter. */
	std::vector<std::vector<cv::Point_<T>>> getPoints(const std::vector<unsigned int> &cameras, unsigned int object);

	std::vector<cv::Point3_<T>> getObject(unsigned int);

	const Visibility& visibility();

	/** Get all points. See Points struct. */
	const std::vector<Points>& all() { return points_; }

protected:
	bool hasCamera(unsigned int c);
	void setCamera(unsigned int c);

private:
	int count_;
	Visibility visibility_;
	Points current_;
	std::vector<Points> points_;
	std::vector<std::vector<cv::Point3_<T>>> objects_;

public:
	MSGPACK_DEFINE(count_, visibility_, current_, points_, objects_);
};

/**
 * Same as OpenCV's recoverPose(), but does not assume same intrinsic paramters
 * for both cameras.
 *
 * @todo Write unit tests to check that intrinsic parameters work as expected.
 */
int recoverPose(const cv::Mat &E, const std::vector<cv::Point2d> &_points1,
	const std::vector<cv::Point2d> &_points2, const cv::Mat &_cameraMatrix1,
	const cv::Mat &_cameraMatrix2, cv::Mat &_R, cv::Mat &_t,
	double distanceThresh, cv::Mat &triangulatedPoints);


/** @brief Calibrate camera pair.
 *
 * Alternative to cv::StereoCalibrate.
 *
 * Essential matrix is estimated using all point correspondencies, and pose is
 * calculated with OpenCV's recoverPose() (modification to allow different
 * intrinsic parameters for each camera).
 *
 * Non-linear optimization is used to
 * determine scale from object points and bundle adjustment is applied to points
 * and extrisnic parameters. Calibration target shape is also included.
 *
 * @param	K1		intrinsic matrix for first camera
 * @param	D1		distortion coefficients for first camera
 * @param	K2		intrinsic matrix for second camera
 * @param	D2		distortion coefficients for second camera
 * @param	points1	image points obeserved in first camera
 * @param	points2	image points observed in second camera
 * @param	object	calibration target points (once)
 * @param	R		(out) rotation matrix (camera 1 to 2)
 * @param	t		(out) translation vector (camera 1 to 2)
 * @param	points_out	triangulated points
 * @param	optimize	optimize points
 *
 * @returns	RMS reprojection error
 *
 * Following conditions must hold for input parameters: (points1.size() ==
 * points2.size()) and (points1.size() % object_points.size() == 0).
 */
double calibratePair(const cv::Mat &K1, const cv::Mat &D1,
	const cv::Mat &K2, const cv::Mat &D2, const std::vector<cv::Point2d> &points1,
	const std::vector<cv::Point2d> &points2, const std::vector<cv::Point3d> &object_points,
	cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &points_out, bool optimize=true);

class ExtrinsicCalibration {
public:

	/** add a single camera. Returns index of camera. */
	unsigned int addCamera(const CalibrationData::Intrinsic &);
	/** Add a stereo camera pair. Pairs always use other cameras to estimate
	 * initial pose. Returns index of first camera. */
	unsigned int addStereoCamera(const CalibrationData::Intrinsic &, const CalibrationData::Intrinsic &);
	/** Add stereo camera pair with initial pose. Returns index of first camera. */
	unsigned int addStereoCamera(const CalibrationData::Intrinsic &, const CalibrationData::Intrinsic &, cv::Vec3d rvec, cv::Vec3d tvec);

	const CalibrationData::Intrinsic& intrinsic(unsigned int c);
	const CalibrationData::Extrinsic& extrinsic(unsigned int c);
	const CalibrationData::Calibration& calibration(unsigned int c);
	const CalibrationData::Calibration& calibrationOptimized(unsigned int c);

	/** Add points/targets; Only one calibration target supported!
	 *
	 * TODO: Support multiple calibration targets: calibrate pair without
	 * optimization or support multiple calibration objects there. Input at the
	 * moment is flat vector, need to group by calibration target size (similar
	 * to cv::stereoCalibrate/cv::calibrateCamera).
	 */
	CalibrationPoints<double>& points() { return points_; }

	/* set bundle adjustment options */
	void setOptions(ftl::calibration::BundleAdjustment::Options options) { options_ = options; }
	ftl::calibration::BundleAdjustment::Options options() { return options_; }

	/** Number of cameras added */
	unsigned int camerasCount() { return calib_.size(); }

	/** status message */
	std::string status();

	/** run calibration, returns reprojection error */
	double run();

	double reprojectionError(unsigned int c); // reprojection error rmse
	double reprojectionError(); // total reprojection error rmse

	/** debug methods */
	bool fromFile(const std::string& fname);
	bool toFile(const std::string& fname); // should return new instance...

	MSGPACK_DEFINE(points_, mask_, pairs_, calib_);

protected:
	/** Initial pairwise calibration and triangulation. */
	void calculatePairPoses();

	/** Calculate initial poses from pairs */
	void calculateInitialPoses();

	/** Bundle adjustment on initial poses and triangulations. */
	double optimize();

private:
	void updateStatus_(std::string);
	std::vector<CalibrationData::Calibration> calib_;
	std::vector<CalibrationData::Calibration> calib_optimized_;
	ftl::calibration::BundleAdjustment::Options options_;

	CalibrationPoints<double> points_;
	std::set<std::pair<unsigned int, unsigned int>> mask_;
	std::map<std::pair<unsigned int, unsigned int>, std::tuple<cv::Mat, cv::Mat, double>> pairs_;
	int min_points_ = 64; // minimum number of points required for pair calibration
	// TODO: add map {c1,c2} for existing calibration which is used if available.
	//
	std::shared_ptr<std::string> status_;

	std::vector<double> rmse_;
	double rmse_total_;

	// Theshold for point to be skipped (m); absdiff between minimum and maximum
	// values of each coordinate axis in all triangulated points is calculated
	// and l2 norm is compared against threshold value. Optimization uses median
	// coordinate values; threshold can be fairly big.
	static constexpr float threshold_bad_ = 0.67;
	// theshold for warning message (% of points discarded)
	static constexpr float threhsold_warning_ = 0.1;
};


} // namespace calibration
}
