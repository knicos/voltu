#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>

#include <ftl/exception.hpp>
#include <ftl/calibration/structures.hpp>
#include <ftl/calibration/parameters.hpp>

using ftl::calibration::CalibrationData;

CalibrationData::Intrinsic::DistortionCoefficients::DistortionCoefficients() :
		data_(14, 0.0) {
}

const cv::Mat CalibrationData::Intrinsic::DistortionCoefficients::Mat(int nparams) const {
	if (nparams <= 0) {
		return cv::Mat();
	}
	if (nparams > 14) {
		nparams = 14;
	}
	return cv::Mat(cv::Size(nparams, 1), CV_64FC1, const_cast<double*>(data_.data()));
}

cv::Mat CalibrationData::Intrinsic::DistortionCoefficients::Mat(int nparams) {
	if (nparams <= 0) {
		return cv::Mat();
	}
	if (nparams > 14) {
		nparams = 14;
	}
	return cv::Mat(cv::Size(nparams, 1), CV_64FC1, data_.data());
}

double& CalibrationData::Intrinsic::DistortionCoefficients::operator[](unsigned i) { return data_[i]; }
double CalibrationData::Intrinsic::DistortionCoefficients::operator[](unsigned i) const { return data_[i]; }

CalibrationData::Intrinsic::Intrinsic() :
	resolution(0, 0), fx(0.0), fy(0.0), cx(0.0), cy(0.0) {}

void CalibrationData::Intrinsic::set(const cv::Mat& K, cv::Size sz) {
	fx = K.at<double>(0, 0);
	fy = K.at<double>(1, 1);
	cx = K.at<double>(0, 2);
	cy = K.at<double>(1, 2);
	resolution = sz;
}

void CalibrationData::Intrinsic::set(const cv::Mat& K, const cv::Mat& D, cv::Size sz) {
	D.copyTo(distCoeffs.Mat(D.cols));
	set(K, sz);
}

CalibrationData::Intrinsic::Intrinsic(const cv::Mat &K, cv::Size size) {
	set(K, size);
}

CalibrationData::Intrinsic::Intrinsic(const cv::Mat &K, const cv::Mat &D, cv::Size size) {
	set(K, D, size);
}

CalibrationData::Intrinsic::Intrinsic(const CalibrationData::Intrinsic& other, cv::Size size) {
	distCoeffs = DistortionCoefficients(other.distCoeffs);
	sensorSize = other.sensorSize;
	auto K = other.matrix(size);
	fx = K.at<double>(0, 0);
	fy = K.at<double>(1, 1);
	cx = K.at<double>(0, 2);
	cy = K.at<double>(1, 2);
	resolution = size;
}

cv::Mat CalibrationData::Intrinsic::matrix() const {
	cv::Mat K(cv::Size(3, 3), CV_64FC1, cv::Scalar(0));
	K.at<double>(0, 0) = fx;
	K.at<double>(1, 1) = fy;
	K.at<double>(0, 2) = cx;
	K.at<double>(1, 2) = cy;
	K.at<double>(2, 2) = 1.0;
	return K;
}

cv::Mat CalibrationData::Intrinsic::matrix(cv::Size size) const {
	return ftl::calibration::scaleCameraMatrix(matrix(), resolution, size);
}

////////////////////////////////////////////////////////////////////////////////


void CalibrationData::Extrinsic::set(const cv::Mat& T) {
	if (T.type() != CV_64FC1) {
		throw ftl::exception("Input must be CV_64FC1");
	}
	if (!ftl::calibration::validate::pose(T)) {
		throw ftl::exception("T is not a valid transform matrix");
	}

	cv::Rodrigues(T(cv::Rect(0, 0, 3, 3)), rvec);
	tvec[0] = T.at<double>(0, 3);
	tvec[1] = T.at<double>(1, 3);
	tvec[2] = T.at<double>(2, 3);
}

void CalibrationData::Extrinsic::set(cv::InputArray R, cv::InputArray t) {
	if ((t.type() != CV_64FC1) || (R.type() != CV_64FC1)) {
		throw ftl::exception("Type of R and t must be CV_64FC1");
	}

	if ((t.size() != cv::Size(3, 1)) && (t.size() != cv::Size(1, 3))) {
		throw ftl::exception("Size of t must be (3, 1) or (1, 3");
	}

	if (R.isMat()) {
		const auto& rmat = R.getMat();

		if (R.size() == cv::Size(3, 3)) {
			if (!ftl::calibration::validate::rotationMatrix(rmat)) {
				throw ftl::exception("R is not a rotation matrix");
			}
			cv::Rodrigues(rmat, rvec);
		}
		else if ((R.size() == cv::Size(3, 1)) || R.size() == cv::Size(1, 3)) {
			rvec[0] = rmat.at<double>(0);
			rvec[1] = rmat.at<double>(1);
			rvec[2] = rmat.at<double>(2);
		}
		else {
			throw ftl::exception("R must be a 3x3 rotation matrix or 3x1/1x3 rotation vector (Rodrigues)");
		}
	}

	const auto& tmat = t.getMat();
	tvec[0] = tmat.at<double>(0);
	tvec[1] = tmat.at<double>(1);
	tvec[2] = tmat.at<double>(2);
}

CalibrationData::Extrinsic::Extrinsic() : rvec(0.0, 0.0, 0.0), tvec(0.0, 0.0, 0.0) {}

CalibrationData::Extrinsic::Extrinsic(const cv::Mat &T) {
	set(T);
}

CalibrationData::Extrinsic::Extrinsic(cv::InputArray R, cv::InputArray t) {
	set(R, t);
}

cv::Mat CalibrationData::Extrinsic::matrix() const {
	cv::Mat T(cv::Size(4, 4), CV_64FC1, cv::Scalar(0));
	cv::Rodrigues(rvec, T(cv::Rect(0, 0, 3, 3)));
	T.at<double>(0, 3) = tvec[0];
	T.at<double>(1, 3) = tvec[1];
	T.at<double>(2, 3) = tvec[2];
	T.at<double>(3, 3) = 1.0;
	return T;
}

 CalibrationData::Extrinsic CalibrationData::Extrinsic::inverse() const {
	return CalibrationData::Extrinsic(ftl::calibration::transform::inverse(matrix()));
}

cv::Mat CalibrationData::Extrinsic::rmat() const {
	cv::Mat R(cv::Size(3, 3), CV_64FC1, cv::Scalar(0));
	cv::Rodrigues(rvec, R);
	return R;
}

////////////////////////////////////////////////////////////////////////////////

CalibrationData CalibrationData::readFile(const std::string &path) {

	cv::FileStorage fs;
	fs.open(path.c_str(), cv::FileStorage::READ);
	if (!fs.isOpened()) {
		throw std::exception();
	}
	CalibrationData calibration;
	fs["enabled"] >> calibration.enabled;

	for (auto it = fs["calibration"].begin(); it != fs["calibration"].end(); it++) {
		Calibration calib;
		ftl::codecs::Channel channel;

		(*it)["channel"] >> channel;
		(*it)["resolution"] >> calib.intrinsic.resolution;
		(*it)["fx"] >> calib.intrinsic.fx;
		(*it)["fy"] >> calib.intrinsic.fy;
		(*it)["cx"] >> calib.intrinsic.cx;
		(*it)["cy"] >> calib.intrinsic.cy;
		(*it)["distCoeffs"] >> calib.intrinsic.distCoeffs.data_;
		(*it)["sensorSize"] >> calib.intrinsic.sensorSize;
		(*it)["rvec"] >> calib.extrinsic.rvec;
		(*it)["tvec"] >> calib.extrinsic.tvec;

		calibration.data_[channel] = calib;
	}

	return calibration;
}

void CalibrationData::writeFile(const std::string &path) const {
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	if (!fs.isOpened()) {
		throw std::exception();
	}

	fs << "enabled" << enabled;
	fs << "calibration" << "[";
	for (auto &[channel, data] : data_) {
		fs	<< "{"
			<<	"channel" << int(channel)
			<<	"resolution" << data.intrinsic.resolution
			<<	"fx" << data.intrinsic.fx
			<<	"fy" << data.intrinsic.fy
			<<	"cx" << data.intrinsic.cx
			<<	"cy" << data.intrinsic.cy
			<<	"distCoeffs" << data.intrinsic.distCoeffs.data_
			<< 	"rvec" << data.extrinsic.rvec
			<< 	"tvec" << data.extrinsic.tvec
			<< 	"sensorSize" << data.intrinsic.sensorSize
			<< "}";
	}
	fs << "]";

	fs.release();
}

CalibrationData::Calibration& CalibrationData::get(ftl::codecs::Channel channel) {
	return data_[channel];
}

bool CalibrationData::hasCalibration(ftl::codecs::Channel channel) const {
	return data_.count(channel) != 0;
}
