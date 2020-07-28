#include <loguru.hpp>

#include <ftl/exception.hpp>
#include <ftl/calibration/object.hpp>

#include <opencv2/core/cuda.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using ftl::calibration::ArUCoObject;
using ftl::calibration::ChessboardObject;


ArUCoObject::ArUCoObject(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary,
	float baseline, float tag_size, int id1, int id2) :
	baseline_(baseline), tag_size_(tag_size),id1_(id1), id2_(id2) {

	dict_ = cv::aruco::getPredefinedDictionary(dictionary);
	params_ = cv::aruco::DetectorParameters::create();
	params_->cornerRefinementMinAccuracy = 0.01;
	// cv::aruco::CORNER_REFINE_CONTOUR memory issues? intrinsic quality?
	params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
}

std::vector<cv::Point3d> ArUCoObject::object() {
	return {
		cv::Point3d(0.0, 0.0, 0.0),
		cv::Point3d(tag_size_, 0.0, 0.0),
		cv::Point3d(tag_size_, tag_size_, 0.0),
		cv::Point3d(0.0, tag_size_, 0.0),

		cv::Point3d(baseline_, 0.0, 0.0),
		cv::Point3d(baseline_ + tag_size_, 0.0, 0.0),
		cv::Point3d(baseline_ + tag_size_, tag_size_, 0.0),
		cv::Point3d(baseline_, tag_size_, 0.0)
	};
}

int ArUCoObject::detect(cv::InputArray im, std::vector<cv::Point2d>& result, const cv::Mat& K, const cv::Mat& distCoeffs) {

	// note: cv::aruco requires floats
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> ids;
	cv::Mat im_gray;

	if (im.size() == cv::Size(0, 0)) {
		return -1;
	}
	if (im.isGpuMat()) {
		cv::Mat dl;
		im.getGpuMat().download(dl);
		cv::cvtColor(dl, im_gray, cv::COLOR_BGRA2GRAY);
	}
	else if (im.isMat()) {
		cv::cvtColor(im.getMat(), im_gray, cv::COLOR_BGRA2GRAY);
	}
	else {
		throw ftl::exception("Bad input (not cv::Mat/cv::GpuMat)");
	}

	cv::aruco::detectMarkers(im_gray, dict_, corners, ids, params_,
								cv::noArray(), K, distCoeffs);


	if (ids.size() < 2) { return 0; }

	const size_t n_corners = 4;
	const size_t n_tags = 2;

	std::vector<cv::Point2d> marker1; marker1.reserve(n_corners);
	std::vector<cv::Point2d> marker2; marker2.reserve(n_corners);

	int n = 0;
	// find the right markers
	for (unsigned int i = 0; i < ids.size(); i++) {
		if (ids[i] == id1_) {
			n++;
			for (auto& p : corners[i]) {
				marker1.push_back({p.x, p.y});
			}
			CHECK(corners[i].size() == n_corners);
		}
		if (ids[i] == id2_) {
			n++;
			for (auto& p : corners[i]) {
				marker2.push_back({p.x, p.y});
			}
			CHECK(corners[i].size() == n_corners);
		}
	}

	if (marker1.empty() || marker2.empty()) {
		return 0;
	}

	if (n != 2) {
		LOG(WARNING) << "Found more than one marker with same ID";
		return 0;
	}

	// add the points to output
	result.reserve(n_tags*n_corners + result.size());
	for (size_t i_corner = 0; i_corner < n_corners; i_corner++) {
		result.push_back(marker1[i_corner]);
	}
	for (size_t i_corner = 0; i_corner < n_corners; i_corner++) {
		result.push_back(marker2[i_corner]);
	}

	return n_tags*n_corners;
}

////////////////////////////////////////////////////////////////////////////////

cv::Size ChessboardObject::chessboardSize() {
	return {chessboard_size_.width + 1, chessboard_size_.height + 1};
}

double ChessboardObject::squareSize() {
	return square_size_;
}

ChessboardObject::ChessboardObject(int rows, int cols, double square_size) :
		chessboard_size_(cols - 1, rows - 1), square_size_(square_size),
		flags_(cv::CALIB_CB_NORMALIZE_IMAGE|cv::CALIB_CB_ACCURACY) {

	init();
}

void ChessboardObject::init() {
	object_points_.reserve(chessboard_size_.width * chessboard_size_.height);
	for (int row = 0; row < chessboard_size_.height; ++row) {
	for (int col = 0; col < chessboard_size_.width; ++col) {
		object_points_.push_back({col * square_size_, row * square_size_, 0});
	}}
}

int ChessboardObject::detect(cv::InputArray im, std::vector<cv::Point2d>& points, const cv::Mat& K, const cv::Mat& D) {
	cv::Mat tmp;

	if (im.isMat()) {
		tmp = im.getMat();
	}
	else if (im.isGpuMat()) {
		im.getGpuMat().download(tmp);
	}
	else {
		throw ftl::exception("Image not cv::Mat or cv::GpuMat");
	}

	if (cv::findChessboardCornersSB(tmp, chessboard_size_, points, flags_)) {
		return 1;
	}
	return 0;
}

std::vector<cv::Point3d> ChessboardObject::object() {
	return object_points_;
}
