#ifndef _FTL_CALIBRATION_STRUCTURES_HPP_
#define _FTL_CALIBRATION_STRUCTURES_HPP_

#include <ftl/utility/msgpack.hpp>
#include <ftl/codecs/channels.hpp>

namespace ftl {
namespace calibration {

struct CalibrationData {

	struct Intrinsic {
		friend CalibrationData;

		/** 12 distortion coefficients. OpenCV also provides tilted camera model
		 * coefficients, but not used here. */
		struct DistortionCoefficients {
			friend CalibrationData;

			DistortionCoefficients();

			/**
			 * Access distortion coefficients, stored in OpenCV order. Out of
			 * bounds access is undefined.
			 *
			 * 0,1			r1-r2 radial distortion
			 * 2,3			p1-p2 tangential distortion
			 * 4,5,6,7		r3-r6 radial distortion
			 * 8,9,10,11	s1-s4 thin prism distortion
			 *
			 */
			double& operator[](unsigned i);
			double operator[](unsigned i) const;

			/** are radial distortion values are for rational model */
			bool rationalModel() const;
			/** is thin prism model is used (s1-s4 set) */
			bool thinPrism() const;

			/**
			 * Return distortion parameters in cv::Mat. Shares same memory.
			 */
			const cv::Mat Mat(int nparams = 12) const;
			cv::Mat Mat(int nparams = 12);

		private:
			std::vector<double> data_;

		public:
			MSGPACK_DEFINE(data_);
		};
		Intrinsic();
		Intrinsic(const cv::Mat &K, cv::Size sz);
		Intrinsic(const cv::Mat &K, const cv::Mat &D, cv::Size sz);

		/** New instance with scaled values for new resolution */
		Intrinsic(const Intrinsic& other, cv::Size sz);

		/* valid values (resolution is non-zero) */
		bool valid() const;

		/** horizontal field of view in degrees */
		double fovx() const;
		/** vertical field of view in degrees */
		double fovy() const;
		/** focal length in sensor size units */
		double focal() const;
		/** aspect ratio: fx/fy */
		double aspectRatio() const;

		/** Replace current values with new ones */
		void set(const cv::Mat &K, cv::Size sz);
		void set(const cv::Mat &K, const cv::Mat &D, cv::Size sz);

		/** Camera matrix */
		cv::Mat matrix() const;
		/** Camera matrix (scaled) */
		cv::Mat matrix(cv::Size) const;

		cv::Size resolution;
		double fx;
		double fy;
		double cx;
		double cy;
		DistortionCoefficients distCoeffs;

		/** (optional) sensor size; Move elsehwere? */
		cv::Size2d sensorSize;

		MSGPACK_DEFINE(resolution, fx, fy, cx, cy, distCoeffs, sensorSize);
	};
	struct Extrinsic {
		Extrinsic();
		Extrinsic(const cv::Mat &T);
		Extrinsic(cv::InputArray R, cv::InputArray t);

		void set(const cv::Mat &T);
		void set(cv::InputArray R, cv::InputArray t);

		Extrinsic inverse() const;

		/** valid calibration (values not NAN) */
		bool valid() const;

		/** get as a 4x4 matrix */
		cv::Mat matrix() const;
		/** get 3x3 rotation matrix */
		cv::Mat rmat() const;

		cv::Vec3d rvec = {NAN, NAN, NAN};
		cv::Vec3d tvec = {NAN, NAN, NAN};
		MSGPACK_DEFINE(rvec, tvec);
	};

	struct Calibration {
		Intrinsic intrinsic;
		Extrinsic extrinsic;

		/** 4x4 projection matrix */
		cv::Mat matrix();

		MSGPACK_DEFINE(intrinsic, extrinsic);
	};

	CalibrationData() : enabled(false) {}
	bool enabled;

	[[nodiscard]]
	static CalibrationData readFile(const std::string &path);
	void writeFile(const std::string &path) const;

	/** Get reference for channel. Create if doesn't exist. */
	Calibration& get(ftl::codecs::Channel channel);
	bool hasCalibration(ftl::codecs::Channel channel) const;

	// TODO: identify cameras with unique ID string instead of channel.
	std::map<ftl::codecs::Channel, Calibration> data;

	/** Correction to be applied (inverse) to extrinsic parameters
	 * (calibrated to new origin); Applied to rectified pose at the moment
	 */
	cv::Mat origin = cv::Mat::eye(4, 4, CV_64FC1);

public:
	MSGPACK_DEFINE(enabled, data, origin);
};

}
}

#endif
