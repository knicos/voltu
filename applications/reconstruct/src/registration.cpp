#include <ftl/registration.hpp>

#ifdef HAVE_PCL

#include <glog/logging.h>
#include <pcl/common/transforms.h>

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>

#include <pcl/registration/transformation_validation.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/common/geometry.h>
//#include <pcl/registration/icp_nl.h>

namespace ftl {
namespace registration {

using ftl::rgbd::CameraParameters;
using std::vector;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGB;

using cv::Mat;

// todo template: fitPlane<typename T>(PointCloud<T> cloud_in, PointCloud<T> cloud_out)
//
// Fit calibration pattern into plane using RANSAC + project points
//
pcl::ModelCoefficients::Ptr fitPlane(PointCloud<PointXYZ>::Ptr cloud_in, float distance_threshold=5.0) {
	// TODO: include pattern in model (find best alignment of found points and return transformed reference?)
	
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	
	// Estimate plane with RANSAC
	pcl::SACSegmentation<PointXYZ> seg;
	
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	seg.setInputCloud(cloud_in);
	seg.segment(*inliers, *coefficients);
	
	return coefficients;
}

float fitPlaneError(PointCloud<PointXYZ>::Ptr cloud_in, float distance_threshold=5.0) {
	auto coefficients = fitPlane(cloud_in, distance_threshold);
	PointCloud<PointXYZ> cloud_proj;
	
	// Project points into plane
	pcl::ProjectInliers<PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_in);
	proj.setModelCoefficients(coefficients);
	proj.filter(cloud_proj); 
	
	LOG_ASSERT(cloud_in->size() == cloud_proj.size());
	
	// todo: which error score is suitable? (using MSE)
	float score = 0.0;
	for(size_t i = 0; i < cloud_proj.size(); i++) {
		float d = pcl::geometry::distance(cloud_in->points[i], cloud_proj.points[i]);
		score += d * d;
	}
	
	return score / cloud_proj.size();
}

//template<typename T = PointXYZ> typename
PointCloud<PointXYZ>::Ptr cornersToPointCloud(const vector<cv::Point2f> &corners, const Mat &disp, const CameraParameters &p) {
	
	int corners_len = corners.size();
	vector<cv::Vec3f> points(corners_len);

	const double CX = p.cx;
	const double CY = p.cy;
	const double FX = p.fx;
	const double FY = p.fy;
	
	// Output point cloud
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	cloud->width = corners_len;
	cloud->height = 1;
	
	// Follows cv::reprojectImageTo3D(..)
	// https://github.com/opencv/opencv/blob/371bba8f54560b374fbcd47e7e02f015ac4969ad/modules/calib3d/src/calibration.cpp#L2998
	// documentation suggests using cv::perspectiveTransform(...) with sparse set of points
	
	for (int i = 0; i < corners_len; i++) {
		double x = corners[i].x;
		double y = corners[i].y;
		double d = disp.at<float>((int) y, (int) x); // * 1000.0f; // todo: better estimation
		
		//cv::Vec4d homg_pt = Q_ * cv::Vec4d(x, y, d, 1.0);
		//cv::Vec3d p = cv::Vec3d(homg_pt.val) / homg_pt[3];

		PointXYZ point;
		point.x = (((double)x + CX) / FX) * d; // / 1000.0f;
		point.y = (((double)y + CY) / FY) * d; // / 1000.0f;
		point.z = d;
		
		// no check since disparities assumed to be good in the calibration pattern
		// if (fabs(d-minDisparity) <= FLT_EPSILON)
		
		cloud->push_back(point);
	}
	
	return cloud;
}

bool findChessboardCorners(Mat &rgb, const Mat &disp, const CameraParameters &p, const cv::Size pattern_size, PointCloud<PointXYZ>::Ptr &out, float error_threshold) {
	vector<cv::Point2f> corners(pattern_size.width * pattern_size.height);

#if CV_VERSION_MAJOR >= 4
	bool retval = cv::findChessboardCornersSB(rgb, pattern_size, corners);
#else
	bool retval = cv::findChessboardCorners(rgb, pattern_size, corners);
#endif

	cv::drawChessboardCorners(rgb, pattern_size, Mat(corners), retval);
	if (!retval) { return false; }
	
	auto corners_cloud = cornersToPointCloud(corners, disp, p);
	// simple check that the values make some sense
	float error = fitPlaneError(corners_cloud, error_threshold); // should use different parameter?
	LOG(INFO) << "MSE against estimated plane: " << error;
	
	if (error > error_threshold) {
		LOG(WARNING) << "too high error score for calibration pattern, threshold " << error_threshold;
		return false;
	}
	
	if (out) { *out += *corners_cloud; } // if cloud is valid, add the points
	else { out = corners_cloud; }
	return true;
}

Eigen::Matrix4f findTransformation(vector<PointCloud<PointXYZ>::Ptr> clouds_source, vector<PointCloud<PointXYZ>::Ptr> clouds_target) {
	size_t n_clouds = clouds_source.size();
	
	Eigen::Matrix4f T, T_tmp, T_new;
	T.setIdentity();
	
	if ((clouds_source.size() != clouds_target.size()) || (n_clouds == 0)) {
		LOG(ERROR) << "Input vectors have invalid sizes: clouds_source " << clouds_source.size()
					<< ", clouds_target " << clouds_target.size() << ", transformation can not be estimated";
		
		return T; // identity
	}
	
	// corresponding points have same indices (!!!)
	int n_points = clouds_source[0]->width * clouds_source[0]->height;
	vector<int> idx(n_points);
	for (int i = 0; i < n_points; i++) { idx[i] = i; }
	
	pcl::registration::TransformationValidationEuclidean<PointXYZ, PointXYZ> validate;
	pcl::registration::TransformationEstimationSVD<PointXYZ,PointXYZ> svd;
	
	double score_prev = std::numeric_limits<float>::max();
	
	for (size_t i = 0; i < n_clouds; ++i) {
		PointCloud<PointXYZ> source;
		PointCloud<PointXYZ> target = *clouds_target[i];
		
		pcl::transformPointCloud(*clouds_source[i], source, T);
		svd.estimateRigidTransformation(source, idx, target, idx, T_new);
		
		// calculate new transformation
		T_tmp = T_new * T;
		
		// score new transformation
		double score = 0.0;
		for (size_t j = 0; j < n_clouds; ++j) {
			score += validate.validateTransformation(clouds_source[j], clouds_target[j], T);
		}
		score /= n_clouds;
		
		// if score doesn't improve, do not use as T, otherwise update T and score
		if (score < score_prev) {
			T = T_tmp;
			score_prev = score;
		}
		
		LOG(INFO) << "Validation score: " << score;
	}
	
	return T;
}

} // namespace registration
} // namespace ftl

#endif // HAVE_PCL