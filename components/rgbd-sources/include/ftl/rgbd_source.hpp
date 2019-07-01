#pragma once
#ifndef _FTL_RGBD_SOURCE_HPP_
#define _FTL_RGBD_SOURCE_HPP_

#include <ftl/config.h>
#include <ftl/configurable.hpp>
#include <ftl/threads.hpp>
#include <ftl/camera_params.hpp>
#include <ftl/net/universe.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

namespace ftl {
namespace rgbd {

/**
 * Abstract class for any generic RGB-Depth data source. It can also store pose
 * information, although this must be provided by an external source.
 */
class RGBDSource : public ftl::Configurable {
	public:
	explicit RGBDSource(nlohmann::json &config);
	RGBDSource(nlohmann::json &config, ftl::net::Universe *net);
	virtual ~RGBDSource();

	virtual void grab()=0;
	virtual bool isReady();

	void getRGBD(cv::Mat &rgb, cv::Mat &depth);

	const CameraParameters &getParameters() { return params_; };
	std::string getURI() const { return config_["$id"].get<std::string>(); }

	virtual void setPose(const Eigen::Matrix4f &pose) { pose_ = pose; };
	const Eigen::Matrix4f &getPose() { return pose_; };

	virtual void reset() {}

	/**
	 * Get a point in camera coordinates at specified pixel location.
	 */
	Eigen::Vector4f point(uint x, uint y);

	/**
	 * Save the current RGB and Depth images to image files (jpg and png) with
	 * the specified file prefix (excluding file extension).
	 */
	bool snapshot(const std::string &fileprefix);

	/**
	 * Generate a video of this RGB-D source.
	 */
	//bool record(const std::string &filename);

	/**
	 * Factory registration class.
	 */
	class Register {
		public:
		// cppcheck-suppress *
		Register(const std::string &n, std::function<RGBDSource*(nlohmann::json&,ftl::net::Universe*)> f) {
			RGBDSource::_register(n,f);
		};
	};
	
	/**
	 * Factory instance creator where config contains a "type" property
	 * used as the instance name to construct.
	 */
	static RGBDSource *create(nlohmann::json &config, ftl::net::Universe *net);

	static void init();
	
	protected:
	static void _register(const std::string &n, std::function<RGBDSource*(nlohmann::json&,ftl::net::Universe*)> f);

	protected:
	CameraParameters params_;
	ftl::net::Universe *net_;
	MUTEX mutex_;
	cv::Mat rgb_;
	cv::Mat depth_;

	private:
	Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();

	private:
	static std::map<std::string,std::function<RGBDSource*(nlohmann::json&,ftl::net::Universe*)>> *sources__;
};

};
};

#endif  // _FTL_RGBD_SOURCE_HPP_
