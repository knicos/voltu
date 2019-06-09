#ifndef _FTL_RECONSTRUCT_REGISTRATION_HPP_
#define _FTL_RECONSTRUCT_REGISTRATION_HPP_

#include <ftl/config.h>
#include <ftl/configurable.hpp>
#include <ftl/rgbd.hpp>
#include <opencv2/opencv.hpp>

#ifdef HAVE_PCL

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>

namespace ftl {
namespace registration {

void to_json(nlohmann::json &json, std::map<std::string, Eigen::Matrix4f> &transformations);
void from_json(nlohmann::json &json, std::map<std::string, Eigen::Matrix4f> &transformations);

bool loadTransformations(const std::string &path, std::map<std::string, Eigen::Matrix4f> &data);
bool saveTransformations(const std::string &path, std::map<std::string, Eigen::Matrix4f> &data);

/** @brief	Find transformation matrix for transforming clouds_source to clouds_target.
 *			Assumes that corresponding points in clouds_source[i] and clouds_target[i] have same indices.
 */
Eigen::Matrix4f findTransformation(	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_source,
									std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_target);


/** @brief Convert chessboard corners found with OpenCV's findChessboardCorners to PCL point cloud. */
pcl::PointCloud<pcl::PointXYZ>::Ptr cornersToPointCloud(const std::vector<cv::Point2f> &corners, const cv::Mat &disp, const ftl::rgbd::Camera &p);

/** @brief 	Find chessboard corners from image and store them in PCL PointCloud.
 * 	@note	Corners will be drawn in rgb.
 */
bool findChessboardCorners(cv::Mat &rgb, const cv::Mat &depth, const ftl::rgbd::Camera &p, const cv::Size pattern_size, pcl::PointCloud<pcl::PointXYZ>::Ptr &out, float error_threshold);

/**
 * @brief	Abstract class for registration
 * 
 * Registration procedure 
 * 
 * @todo	Support for multiple features (possibly necessary for other algorithms).
 */
class Registration : public ftl::Configurable {
public:
	explicit Registration(nlohmann::json &config);
	void addSource(ftl::rgbd::Source* source);
	size_t getSourcesCount() { return sources_.size(); }

	/**
	 * @brief	Run registration loop. 
	 * 
	 * Loop terminates when processData() returns false.
	 */
	virtual void run();

	/**
	 * @brief	Find registration transformations. run() must be called before
	 * 			findTransformations(). Transformations are in same order as
	 * 			sources were added with addSource().
	 * 
	 * Transformations are calculated to targetsource if configured. If
	 * targetsource is not configured or is not found in inputs, which target
	 * coordinate system is used depends on sub-classes' implementation.
	 * 
	 * @param	Output parameter for transformations.
	 * @return	True if transformations found, otherwise false.
	 */
	virtual bool findTransformations(std::vector<Eigen::Matrix4f> &data)=0;
	
	/**
	 * @brief	Overload of findTransformations(). Map keys are source URIs.
	 */
	virtual bool findTransformations(std::map<std::string, Eigen::Matrix4f> &data);

protected:
	ftl::rgbd::Source* getSource(size_t idx);

	bool isTargetSourceSet();
	bool isTargetSourceFound();
	bool isTargetSource(ftl::rgbd::Source* source); 
	bool isTargetSource(size_t idx); 
	
	/**
	 * @brief	Get index for target source. If target source not defined
	 * 			returns 0. If target source is not found, returns 0.
	 */
	size_t getTargetSourceIdx();
	
	/**
	 * @brief	Resets visibility matrix.
	 */
	void resetVisibility();

	/**
	 * @brief	Check if there is enough data to cover all the cameras;
	 * 			that is visibility is a connected graph. Implemented with BFS.
	 * 			Implementations of processData() in sub-classes may use this
	 * 			method (but it is not required).
	 * 
	 * @todo	Add support for checking visibility for each different features
	 * 			found in all images. Also see findFeatures().
	 */
	bool connectedVisibility();

	/**
	 * @brief	Check if there are enough data for all the sources to calculate
	 * 			transformations. Method may also implement additional processing
	 * 			for the data. Called once after iteration round. run() stops
	 * 			when processData() returns false.
	 */
	virtual bool processData()=0;

	/**
	 * @brief	Find features in source, return 
	 * 
	 * Called iteratively n times for every input (n * inputs.size()). Exact
	 * details depend on implementation of processData(). Implementations may
	 * choose not to set/use visibility information.
	 * 
	 * @param	Input source
	 * @param	Unique index for source provided by run(). Same source will
	 * 			always have same idx. Implementations may choose to ignore it.
	 * @return	True/false if feature was found. Used to build adjacency matrix.
	 */
	virtual bool findFeatures(ftl::rgbd::Source* source, size_t idx)=0;

	std::vector<std::vector<bool>> visibility_; /*< Adjacency matrix for sources (feature visibility). */

private:
	std::optional<std::string> target_source_; /*< Reference coordinate system for transformations. */
	std::vector<ftl::rgbd::Source*> sources_;
};

/**
 * @brief	Registration using chessboard calibration pattern
 * 
 * Parameters from configuration:
 * 
 * 		patternsize: required
 * 			Chessboard pattern size, inner corners.
 * 
 * 		maxerror: default +inf
 * 			Maximum allowed error value for pattern detection. MSE error between
 * 			estimated plane and points captured from input.
 * 
 * 		delay:	default 500
 * 			Milliseconds between captured images.
 * 
 * 		chain: default false
 * 			Enabling allows camera chaining. In chain mode, pattern is not
 * 			required to be visible in every source. In default (chain: false)
 * 			mode, pattern visibility is required for every source.
 * 
 * 		iter: default 10
 * 			Number of iterations for capturing calibration samples. In
 * 			non-chaining mode, each iteration consists of images where patterns
 * 			were detected on every input. In chaining mode each iteration only
 * 			requires camera visibility to be connected.
 */
class ChessboardRegistration : public Registration {
public:
	explicit ChessboardRegistration(nlohmann::json &config);
	/** 
	 * @brief	Creates new ChessboardRegistration or ChessboardRegistrationChain
	 * 			object depending on chain option in config. User of the method
	 * 			needs to free the memory.
	 */
	static ChessboardRegistration* create(nlohmann::json &config);

	void run() override;
	bool findTransformations(std::vector<Eigen::Matrix4f> &data) override;

protected:
	bool findFeatures(ftl::rgbd::Source* source, size_t idx) override;
	bool processData() override;
	cv::Size pattern_size_;
	std::vector<std::vector<std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr>>> data_;
	std::vector<Eigen::Matrix4f> T_;
	float error_threshold_;
	uint delay_;
	uint iter_;
	uint iter_remaining_;
};

/**
 * @brief Chain registration. Finds visibility and then runs registration.
 */
class ChessboardRegistrationChain : public ChessboardRegistration {
public:
	explicit ChessboardRegistrationChain(nlohmann::json &config);
	
	bool findTransformations(std::vector<Eigen::Matrix4f> &data) override;

protected:
	bool processData() override;
	std::vector<Eigen::Matrix4f> T_;
	std::vector<std::vector<std::pair<size_t, size_t>>> edges_;
};

}
}

#endif  // HAVE_PCL
#endif  // _FTL_RECONSTRUCT_REGISTRATION_HPP_