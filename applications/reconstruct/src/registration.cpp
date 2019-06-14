#include <ftl/registration.hpp>

#ifdef HAVE_PCL

#define LOGURU_WITH_STREAMS 1
#include <loguru.hpp>
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

using ftl::rgbd::Camera;
using ftl::rgbd::Source;

using std::string;
using std::vector;
using std::pair;
using std::map;
using std::optional;

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGB;

using cv::Mat;
using Eigen::Matrix4f;

void from_json(nlohmann::json &json, map<string, Matrix4f> &transformations) {
	for (auto it = json.begin(); it != json.end(); ++it) {
		Eigen::Matrix4f m;
		auto data = m.data();
		for(size_t i = 0; i < 16; i++) { data[i] = it.value()[i]; }
		transformations[it.key()] = m;
	}
}

void to_json(nlohmann::json &json, map<string, Matrix4f> &transformations) {
	for (auto &item : transformations) {
		auto val = nlohmann::json::array();
		for(size_t i = 0; i < 16; i++) { val.push_back((float) item.second.data()[i]); }
		json[item.first] = val;
	}
}

bool loadTransformations(const string &path, map<string, Matrix4f> &data) {
	std::ifstream file(path);
	if (!file.is_open()) {
		LOG(ERROR) << "Error loading transformations from file " << path;
		return false;
	}
	
	nlohmann::json json_registration;
	file >> json_registration;
	from_json(json_registration, data);
	return true;
}

bool saveTransformations(const string &path, map<string, Matrix4f> &data) {
	nlohmann::json data_json;
	to_json(data_json, data);
	std::ofstream file(path);

	if (!file.is_open()) {
		LOG(ERROR) << "Error writing transformations to file " << path;
		return false;
	}

	file << std::setw(4) << data_json;
	return true;
}

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
	
	CHECK(cloud_in->size() == cloud_proj.size());
	
	// todo: which error score is suitable? (using MSE)
	float score = 0.0;
	for(size_t i = 0; i < cloud_proj.size(); i++) {
		float d = pcl::geometry::distance(cloud_in->points[i], cloud_proj.points[i]);
		score += d * d;
	}
	
	return (score / cloud_proj.size()) * 10000000.0f;
}

//template<typename T = PointXYZ> typename
PointCloud<PointXYZ>::Ptr cornersToPointCloud(const vector<cv::Point2f> &corners, const Mat &depth, const Camera &p) {
	
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
		double d = depth.at<float>((int) y, (int) x); // * 1000.0f; // todo: better estimation
		
		//cv::Vec4d homg_pt = Q_ * cv::Vec4d(x, y, d, 1.0);
		//cv::Vec3d p = cv::Vec3d(homg_pt.val) / homg_pt[3];

		PointXYZ point;
		point.x = (((double)x + CX) / FX) * d; // / 1000.0f;
		point.y = (((double)y + CY) / FY) * d; // / 1000.0f;
		point.z = d;
		
		cloud->push_back(point);
	}
	
	return cloud;
}

bool findChessboardCorners(Mat &rgb, const Mat &depth, const Camera &p, const cv::Size pattern_size, PointCloud<PointXYZ>::Ptr &out, float error_threshold) {
	vector<cv::Point2f> corners(pattern_size.width * pattern_size.height);

#if CV_VERSION_MAJOR >= 4
	bool retval = cv::findChessboardCornersSB(rgb, pattern_size, corners);
#else
	bool retval = cv::findChessboardCorners(rgb, pattern_size, corners);
#endif

	cv::drawChessboardCorners(rgb, pattern_size, Mat(corners), retval);
	if (!retval) { return false; }
	
	auto corners_cloud = cornersToPointCloud(corners, depth, p);
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

Registration::Registration(nlohmann::json &config) :
	ftl::Configurable(config) {
	target_source_ = get<string>("targetsource");
	if (!target_source_) {
		LOG(WARNING) << "targetsource not set";
	}
}

Source* Registration::getSource(size_t idx) {
	return sources_[idx];
}

bool Registration::isTargetSourceSet() {
	return (bool) target_source_;
}

bool Registration::isTargetSourceFound() {
	for (Source* source : sources_ ) {
		if (isTargetSource(source)) return true;
	}
	return false;
}

bool Registration::isTargetSource(Source *source) {
	if (target_source_) { return source->getID() == *target_source_; }
	return false;
}

bool Registration::isTargetSource(size_t idx) {
	if (idx >= sources_.size()) return false; // assert
	return isTargetSource(sources_[idx]);
}

size_t Registration::getTargetSourceIdx() {
	if (!target_source_) return 0;
	for (size_t idx = 0; idx < sources_.size(); ++idx) {
		if (isTargetSource(sources_[idx])) return idx;
	}

	return 0;
}

void Registration::addSource(Source *source) {
	// TODO: check that source is not already included
	sources_.push_back(source);
}

/**
 * @param	adjacency matrix
 * @param	index of starting vertex
 * @param	(out) edges connecting each level
 * @returns	true if graph connected (all vertices visited), otherwise false
 * 
 * Breadth First Search
 */
bool isConnected(vector<vector<bool>> matrix, size_t start_idx, vector<vector<pair<size_t, size_t>>> &edges) {
vector<bool> visited(matrix.size(), false);
	DCHECK(start_idx < matrix.size());

	edges.clear();
	vector<size_t> level { start_idx };

	visited[start_idx] = true;
	size_t visited_count = 1;
	
	while(level.size() != 0) {
		vector<size_t> level_prev = level;
		level = {};

		vector<pair<size_t, size_t>> new_edges;

		for (size_t current : level_prev) {
		for (size_t i = 0; i < matrix.size(); ++i) {
			if (matrix[current][i] && !visited[i]) {
				visited[i] = true;
				visited_count += 1;
				level.push_back(i);
				// could also save each level's vertices

				new_edges.push_back(pair(current, i));
			}
		}}
		if (new_edges.size() > 0) edges.push_back(new_edges);
	}

	return visited_count == matrix.size();
}

bool isConnected(vector<vector<bool>> matrix, size_t start_idx = 0) {
	vector<vector<pair<size_t, size_t>>> edges;
	return isConnected(matrix, start_idx, edges);
}

/**
 * @param	Adjacency matrix
 * @returns	Vector containing degree of each vertex
*/
vector<uint> verticleDegrees(vector<vector<bool>> matrix) {
	vector<uint> res(matrix.size(), 0);
	for (size_t i = 0; i < matrix.size(); ++i) {
	for (size_t j = 0; j < matrix.size(); ++j) {
		if (matrix[i][j]) res[i] = res[i] + 1;
	}}
	return res;
}

bool Registration::connectedVisibility() {
	return isConnected(visibility_, getTargetSourceIdx());
}

void Registration::resetVisibility() {
	visibility_ = vector(sources_.size(), vector<bool>(sources_.size(), false));
}

void Registration::run() {
	resetVisibility();

	do {
		vector<bool> visible(sources_.size(), false);

		for (size_t i = 0; i < sources_.size(); ++i) {
			bool retval = findFeatures(sources_[i], i);
			visible[i] = retval;
		}

		for (size_t i = 0; i < visible.size(); ++i) {
		for (size_t j = 0; j < visible.size(); ++j) {
			bool val = visible[i] && visible[j];
			visibility_[i][j] = visibility_[i][j] || val;
			visibility_[j][i] = visibility_[j][i] || val;
		}}
	}
	while(processData());
}

bool Registration::findTransformations(map<string, Matrix4f> &data) {
	vector<Matrix4f> T;
	data.clear();

	if (!findTransformations(T)) return false;
	for (size_t i = 0; i < sources_.size(); ++i) {
		data[sources_[i]->getID()] = T[i];
	}
	return true;
}

ChessboardRegistration* ChessboardRegistration::create(nlohmann::json &config) {
	if (config.value<bool>("chain", false)) {
		return new ChessboardRegistrationChain(config);
	}
	else {
		return new ChessboardRegistration(config);
	}
}

ChessboardRegistration::ChessboardRegistration(nlohmann::json &config) :
	Registration(config) {
	
	auto patternsize = get<vector<int>>("patternsize");
	if (!patternsize) { LOG(FATAL) << "Registration run enabled but pattern size not set"; }
	pattern_size_ = cv::Size((*patternsize)[0], (*patternsize)[1]);
	
	auto maxerror = get<float>("maxerror");
	if (!maxerror) { LOG(WARNING) << "maxerror not set"; }
	auto delay = get<int>("delay");
	if (!delay) { LOG(INFO) << "delay not set in configuration"; }
	auto iter = get<int>("iterations");
	if (!iter) { LOG(INFO) << "iterations not set in configuration"; }
	auto chain = get<bool>("chain");
	if (!chain) { LOG(INFO) << "input chaining disabled"; }
	else { LOG(INFO) << "Input chaining enabled"; }

	error_threshold_ = maxerror ? *maxerror : std::numeric_limits<float>::infinity();
	iter_ = iter ? *iter : 10;
	delay_ = delay ? *delay : 50;
}

void ChessboardRegistration::run() {
	if (!isTargetSourceFound()) {
		LOG(WARNING) << "targetsource not found in sources";
	}

	if (data_.size() != getSourcesCount()) {
		data_ = vector<vector<optional<PointCloud<PointXYZ>::Ptr>>>(getSourcesCount());
	}
	iter_remaining_ = iter_;

	// TODO: Move GUI elsewhere. Also applies to processData() and findFeatures()
	for (size_t i = 0; i < getSourcesCount(); ++i) { 
		cv::namedWindow("Registration: " + getSource(i)->getID(),
						cv::WINDOW_KEEPRATIO|cv::WINDOW_NORMAL);
	}

	Registration::run();

	for (size_t i = 0; i < getSourcesCount(); ++i) { 
		cv::destroyWindow("Registration: " + getSource(i)->getID());
	}
}

bool ChessboardRegistration::findFeatures(Source *source, size_t idx) {
	optional<PointCloud<PointXYZ>::Ptr> result;
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	Mat rgb, depth;
	source->grab();
	source->getFrames(rgb, depth);

	bool retval = findChessboardCorners(rgb, depth, source->parameters(), pattern_size_, cloud, error_threshold_);
	if (retval) {
		result.emplace(cloud);
	}
	data_[idx].push_back(result);
	
	cv::imshow("Registration: " + source->getID(), rgb);

	return retval;
}

bool ChessboardRegistration::processData() {
	bool retval = connectedVisibility();
	resetVisibility();

	if (retval) {
		iter_remaining_--;
	}
	else{
		LOG(INFO) << "Pattern not visible in all inputs";
		for (auto &sample : data_) { sample.pop_back(); }
	}

	//std::this_thread::sleep_for(std::chrono::milliseconds(delay_));
	cv::waitKey(delay_); // OpenCV GUI doesn't show otherwise

	return iter_remaining_ > 0;
}

bool ChessboardRegistration::findTransformations(vector<Matrix4f> &data) {
	data.clear();
	vector<bool> status(getSourcesCount(), false);
	size_t idx_target = getTargetSourceIdx();

	for (size_t idx = 0; idx < getSourcesCount(); ++idx) {
		Matrix4f T;
		if (idx == idx_target) {
			T.setIdentity(); 
		}
		else {
			vector<PointCloud<PointXYZ>::Ptr> d;
			vector<PointCloud<PointXYZ>::Ptr> d_target;
			d.reserve(iter_);
			d_target.reserve(iter_);
			
			for (size_t i = 0; i < iter_; ++i) {
				auto val = data_[idx][i];
				auto val_target = data_[idx_target][i];
				if (val && val_target) {
					d.push_back(*val);
					d_target.push_back(*val_target);
				}
			}
			T = findTransformation(d, d_target); 
		}
		data.push_back(T);
	}
	return true;
}

ChessboardRegistrationChain::ChessboardRegistrationChain(nlohmann::json &config) :
							ChessboardRegistration(config) {
		error_threshold_ = std::numeric_limits<float>::infinity();
}

bool ChessboardRegistrationChain::processData() {
	for (auto &sample : data_ ) { sample.clear(); }
	bool retval = isConnected(visibility_, getTargetSourceIdx(), edges_);
	
	if (retval) {
		LOG(INFO) << "Chain complete, depth: " << edges_.size();
		return false;
	}
	else{
		LOG(5) << "Chain not complete ";
	}

	return true;
}

bool ChessboardRegistrationChain::findTransformations(vector<Matrix4f> &data) {
	// TODO	Change to group registration: register all sources which have visibility
	//		to the target source in chain.

	LOG(INFO) << "Running pairwise registration";
	data = vector<Matrix4f>(getSourcesCount(), Matrix4f::Identity());

	for (vector<pair<size_t, size_t>> level : edges_) {
		for (pair<size_t, size_t> edge : level) {
			LOG(INFO) 	<< "Registering source "
						<< getSource(edge.second)->getID() << " to source"
						<< getSource(edge.first)->getID();
			
			nlohmann::json conf(config_);
			conf["targetsource"] = getSource(edge.first)->getID();
			conf["chain"] = false;

			vector<Matrix4f> result;
			ChessboardRegistration reg(conf);
			reg.addSource(getSource(edge.first));
			reg.addSource(getSource(edge.second));
			reg.run();
			if (!reg.findTransformations(result)) { return false; }
			data[edge.second] = data[edge.first] * result[1];
		}
	}
	
	return true;
}

} // namespace registration
} // namespace ftl

#endif // HAVE_PCL