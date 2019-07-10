#include "correspondances.hpp"
#include <nlohmann/json.hpp>
#include <random>
#include <chrono>
#include <thread>
#include <opencv2/xphoto.hpp>
#include <pcl/registration/icp.h>

using ftl::registration::Correspondances;
using std::string;
using std::map;
using std::vector;
using ftl::rgbd::Source;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGB;
using nlohmann::json;
using std::tuple;
using std::make_tuple;
using cv::Mat;

void ftl::registration::build_correspondances(const vector<Source*> &sources, map<string, Correspondances*> &cs, int origin, map<string, Eigen::Matrix4d> &old) {
	Correspondances *last = nullptr;

	cs[sources[origin]->getURI()] = nullptr;

	for (int i=origin-1; i>=0; i--) {
		if (last == nullptr) {
			auto *c = new Correspondances(sources[i], sources[origin]);
			last = c;
			cs[sources[i]->getURI()] = c;
			if (old.find(sources[i]->getURI()) != old.end()) {
				c->setTransform(old[sources[i]->getURI()]);
			}
		} else {
			auto *c = new Correspondances(last, sources[i]);
			last = c;
			cs[sources[i]->getURI()] = c;
			if (old.find(sources[i]->getURI()) != old.end()) {
				c->setTransform(old[sources[i]->getURI()]);
			}
		}
	}

	last = nullptr;

	for (int i=origin+1; i<sources.size(); i++) {
		if (last == nullptr) {
			auto *c = new Correspondances(sources[i], sources[origin]);
			last = c;
			cs[sources[i]->getURI()] = c;
		} else {
			auto *c = new Correspondances(last, sources[i]);
			last = c;
			cs[sources[i]->getURI()] = c;
		}
	}
}


Correspondances::Correspondances(Source *src, Source *targ)
	:	parent_(nullptr), targ_(targ), src_(src),
		targ_cloud_(new PointCloud<PointXYZ>),
		src_cloud_(new PointCloud<PointXYZ>), uptodate_(true) {
	transform_.setIdentity();
}

Correspondances::Correspondances(Correspondances *parent, Source *src)
	:	parent_(parent), targ_(parent_->source()), src_(src),
		targ_cloud_(new PointCloud<PointXYZ>),
		src_cloud_(new PointCloud<PointXYZ>), uptodate_(true) {
	transform_.setIdentity();
}

static void averageDepth(vector<Mat> &in, Mat &out, float varThresh) {
	const int rows = in[0].rows;
	const int cols = in[0].cols;
	float varThresh2 = varThresh*varThresh;

	// todo: create new only if out is empty (or wrong shape/type)
	out = Mat(rows, cols, CV_32FC1);

	for (int i = 0; i < rows * cols; ++i) {
		double sum = 0;
		int good_values = 0;

		// Calculate mean
		for (int i_in = 0; i_in < in.size(); ++i_in) {
			double d = in[i_in].at<float>(i);
			if (ftl::rgbd::isValidDepth(d)) {
				good_values++;
				sum += d;
			}
		}

		if (good_values > 0) {
			sum /= (double)good_values;

			// Calculate variance
			double var = 0.0;
			for (int i_in = 0; i_in < in.size(); ++i_in) {
				double d = in[i_in].at<float>(i);
				if (d < 40.0) {
					double delta = (d - sum);
					var += delta*delta;

					//LOG(INFO) << "VAR " << delta;
				}
			}
			if (good_values > 1) var /= (double)(good_values-1);
			else var = 0.0;

			if (var < varThresh2) {
				out.at<float>(i) = (float)sum;
			} else {
				out.at<float>(i) = 0.0f;
			}
		} else {
			out.at<float>(i) = 0.0f;
		}
	}
}

static PointXYZ makePCL(Source *s, int x, int y) {
	Eigen::Vector4d p1 = s->point(x,y);
	PointXYZ pcl_p1;
	pcl_p1.x = p1[0];
	pcl_p1.y = p1[1];
	pcl_p1.z = p1[2];
	return pcl_p1;
}

void Correspondances::drawTarget(cv::Mat &img) {
	using namespace cv;

	for (size_t i=0; i<log_.size(); i++) {
	//for (auto &p : points) {
		auto [tx,ty,sx,sy] = log_[i];
		drawMarker(img, Point(tx,ty), Scalar(0,255,0), MARKER_TILTED_CROSS);
	}
	
	vector<Eigen::Vector2i> tpoints;
	getTransformedFeatures2D(tpoints);
	for (size_t i=0; i<tpoints.size(); i++) {
		Eigen::Vector2i p = tpoints[i];
		drawMarker(img, Point(p[0],p[1]), Scalar(255,0,0), MARKER_TILTED_CROSS);
	}
}

void Correspondances::drawSource(cv::Mat &img) {
	using namespace cv;
	
	for (size_t i=0; i<log_.size(); i++) {
	//for (auto &p : points) {
		auto [tx,ty,sx,sy] = log_[i];
		drawMarker(img, Point(sx,sy), Scalar(0,255,0), MARKER_TILTED_CROSS);
	}
	
	/*vector<Eigen::Vector2i> tpoints;
	getTransformedFeatures2D(tpoints);
	for (size_t i=0; i<tpoints.size(); i++) {
		Eigen::Vector2i p = tpoints[i];
		drawMarker(img, Point(p[0],p[1]), Scalar(255,0,0), MARKER_TILTED_CROSS);
	}*/
}

void Correspondances::clear() {
	targ_cloud_->clear();
	src_cloud_->clear();
	src_index_ = cv::Mat(cv::Size(src_->parameters().width, src_->parameters().height), CV_32SC1, cv::Scalar(-1));
	targ_index_ = cv::Mat(cv::Size(targ_->parameters().width, targ_->parameters().height), CV_32SC1, cv::Scalar(-1));
	src_feat_.clear();
	targ_feat_.clear();
	log_.clear();
}

void Correspondances::clearCorrespondances() {
	src_feat_.clear();
	targ_feat_.clear();
	log_.clear();
	uptodate_ = false;
}

bool Correspondances::capture(cv::Mat &rgb1, cv::Mat &rgb2) {
	Mat d1, d2;
	size_t buffer_size = 10;
	size_t buffer_i = 0;
	vector<vector<Mat>> buffer(2, vector<Mat>(buffer_size));

	for (size_t i = 0; i < buffer_size; ++i) {
		src_->grab();
		targ_->grab();
		src_->getFrames(rgb1, d1);
		targ_->getFrames(rgb2, d2);

		d1.copyTo(buffer[0][i]);
		d2.copyTo(buffer[1][i]);

		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	averageDepth(buffer[0], d1, 0.02f);
	averageDepth(buffer[1], d2, 0.02f);
	Mat d1_v, d2_v;
	d1.convertTo(d1_v, CV_8U, 255.0f / 10.0f);
	d2.convertTo(d2_v, CV_8U, 255.0f / 10.0f);
	applyColorMap(d1_v, d1_v, cv::COLORMAP_JET);
	applyColorMap(d2_v, d2_v, cv::COLORMAP_JET);

	cv::imshow("smooth d1", d1_v);
	cv::imshow("smooth d2", d2_v);

	// Should be done in RGB-Depth source class
	cv::Ptr<cv::xphoto::WhiteBalancer> wb;
	wb = cv::xphoto::createSimpleWB();
	wb->balanceWhite(rgb1, rgb1);
	wb->balanceWhite(rgb2, rgb2);

	// Build point clouds
	clear();
	int six=0;
	int tix=0;

	for (int y=0; y<rgb1.rows; y++) {
		//unsigned char *rgb1ptr = rgb1.ptr(y);
		//unsigned char *rgb2ptr = rgb2.ptr(y);
		float *d1ptr = (float*)d1.ptr(y);
		float *d2ptr = (float*)d2.ptr(y);

		for (int x=0; x<rgb1.cols; x++) {
			float d1_value = d1ptr[x];
			float d2_value = d2ptr[x];

			if (d1_value > 0.1f && d1_value < 39.0f) {
				// Make PCL points with specific depth value
				pcl::PointXYZ p1;
				Eigen::Vector4d p1e = src_->point(x,y,d1_value);
				p1.x = p1e[0];
				p1.y = p1e[1];
				p1.z = p1e[2];
				src_cloud_->push_back(p1);
				src_index_.at<int>(y,x) = six;
				six++;
			}

			if (d2_value > 0.1f && d2_value < 39.0f) {
				// Make PCL points with specific depth value
				pcl::PointXYZ p2;
				Eigen::Vector4d p2e = targ_->point(x,y,d2_value);
				p2.x = p2e[0];
				p2.y = p2e[1];
				p2.z = p2e[2];
				targ_cloud_->push_back(p2);
				targ_index_.at<int>(y,x) = tix;
				tix++;
			}
		}
	}

	return true; // TODO: return statement was missing; is true correct?
}

bool Correspondances::add(int tx, int ty, int sx, int sy) {
	LOG(INFO) << "Adding...";
	int tix = targ_index_.at<int>(ty,tx);
	int six = src_index_.at<int>(sy,sx);

	// Validate feature
	if (tix == -1 || six == -1) {
		LOG(WARNING) << "Bad point";
		return false;
	}

	log_.push_back(make_tuple(tx,ty,sx,sy));

	// Record correspondance point cloud point indexes
	src_feat_.push_back(six);
	targ_feat_.push_back(tix);

	uptodate_ = false;
	LOG(INFO) << "Point added: " << tx << "," << ty << " and " << sx << "," << sy;
	return true;
}

void Correspondances::removeLast() {
	uptodate_ = false;
	targ_feat_.erase(targ_feat_.end()-1);
	src_feat_.erase(src_feat_.end()-1);
	log_.pop_back();
}

double Correspondances::estimateTransform(Eigen::Matrix4d &T) {
	pcl::registration::TransformationValidationEuclidean<PointXYZ, PointXYZ, double> validate;
	pcl::registration::TransformationEstimationSVD<PointXYZ,PointXYZ, double> svd;

	//validate.setMaxRange(0.1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr targ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tsrc_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	vector<int> idx;

	for (int i=0; i<src_feat_.size(); i++) {
		idx.push_back(i);
		src_cloud->push_back(src_cloud_->at(src_feat_[i]));
		targ_cloud->push_back(targ_cloud_->at(targ_feat_[i]));
	}

	pcl::transformPointCloud(*src_cloud, *tsrc_cloud, transform_);

	svd.estimateRigidTransformation(*src_cloud, idx, *targ_cloud, idx, T);
	return validate.validateTransformation(src_cloud, targ_cloud, T);
}

double Correspondances::estimateTransform(Eigen::Matrix4d &T, const std::vector<cv::Vec3d> &src_feat, const std::vector<cv::Vec3d> &targ_feat, bool doicp) {
	pcl::registration::TransformationValidationEuclidean<PointXYZ, PointXYZ, double> validate;
	pcl::registration::TransformationEstimationSVD<PointXYZ,PointXYZ, double> svd;

	pcl::PointCloud<pcl::PointXYZ>::Ptr targ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tsrc_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	vector<int> idx;

	for (int i=0; i<src_feat.size(); i++) {
		pcl::PointXYZ ps,pt;

		ps.x = src_feat[i][0];
		ps.y = src_feat[i][1];
		ps.z = src_feat[i][2];
		pt.x = targ_feat[i][0];
		pt.y = targ_feat[i][1];
		pt.z = targ_feat[i][2];

		idx.push_back(i);
		src_cloud->push_back(ps);
		targ_cloud->push_back(pt);
	}

	pcl::transformPointCloud(*src_cloud, *tsrc_cloud, transform_);

	svd.estimateRigidTransformation(*tsrc_cloud, idx, *targ_cloud, idx, T);

	if (doicp) {
		pcl::transformPointCloud(*tsrc_cloud, *src_cloud, T);

		pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> icp;

		icp.setInputSource(src_cloud);
		icp.setInputTarget(targ_cloud);
		icp.align(*final_cloud);
		LOG(INFO) << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();
		//transform_ *= icp.getFinalTransformation();

		T = icp.getFinalTransformation() * T * transform_;
		//uptodate_ = true;

		return icp.getFitnessScore();
	} else {
		return validate.validateTransformation(src_cloud, targ_cloud, T);
	}
}

double Correspondances::estimateTransform(Eigen::Matrix4d &T, const vector<int> &src_feat, const vector<int> &targ_feat) {
	pcl::registration::TransformationValidationEuclidean<PointXYZ, PointXYZ, double> validate;
	pcl::registration::TransformationEstimationSVD<PointXYZ,PointXYZ, double> svd;

	pcl::PointCloud<pcl::PointXYZ>::Ptr targ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tsrc_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	vector<int> idx;

	for (int i=0; i<src_feat.size(); i++) {
		idx.push_back(i);
		src_cloud->push_back(src_cloud_->at(src_feat[i]));
		targ_cloud->push_back(targ_cloud_->at(targ_feat[i]));
	}

	pcl::transformPointCloud(*src_cloud, *tsrc_cloud, transform_);

	validate.setMaxRange(0.1);

	svd.estimateRigidTransformation(*tsrc_cloud, idx, *targ_cloud, idx, T);
	T = T * transform_;
	uptodate_ = true;
	float score = validate.validateTransformation(src_cloud, targ_cloud, T);
	return score;
}

double Correspondances::icp() {
	//pcl::PointCloud<pcl::PointXYZ>::Ptr tsrc_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr targ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tsrc_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	vector<int> idx;

	for (int i=0; i<src_feat_.size(); i++) {
		idx.push_back(i);
		src_cloud->push_back(src_cloud_->at(src_feat_[i]));
		targ_cloud->push_back(targ_cloud_->at(targ_feat_[i]));
	}

	pcl::transformPointCloud(*src_cloud, *tsrc_cloud, transform_);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> icp;
	icp.setInputSource(tsrc_cloud);
	icp.setInputTarget(targ_cloud);
	icp.align(*final_cloud);
	LOG(INFO) << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();
	transform_ *= icp.getFinalTransformation();
	return icp.getFitnessScore();
}

double Correspondances::icp(const Eigen::Matrix4d &T_in, Eigen::Matrix4d &T_out, const vector<int> &idx) {
	//pcl::PointCloud<pcl::PointXYZ>::Ptr tsrc_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr targ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tsrc_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i=0; i<idx.size(); i++) {
		src_cloud->push_back(src_cloud_->at(src_feat_[idx[i]]));
		targ_cloud->push_back(targ_cloud_->at(targ_feat_[idx[i]]));
	}

	pcl::transformPointCloud(*src_cloud, *tsrc_cloud, T_in);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double> icp;
	icp.setInputSource(tsrc_cloud);
	icp.setInputTarget(targ_cloud);
	icp.align(*final_cloud);
	LOG(INFO) << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();
	T_out = T_in * icp.getFinalTransformation();
	return icp.getFitnessScore();
}

static std::default_random_engine generator;

void Correspondances::convertToPCL(const std::vector<std::tuple<int,int,int,int>> &p, std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &pp) {
	for (size_t i=0; i<p.size(); i++) {
		auto [sx,sy,tx,ty] = p[i];
		//LOG(INFO) << "POINT " << sx << "," << sy;
		auto p1 = makePCL(src_, sx, sy);
		auto p2 = makePCL(targ_, tx, ty);
		pp.push_back(std::make_pair(p1,p2));
	}
}

void Correspondances::getFeatures3D(std::vector<Eigen::Vector4d> &f) {
	for (int i=0; i<src_feat_.size(); i++) {
		Eigen::Vector4d p;
		const pcl::PointXYZ &pp = src_cloud_->at(src_feat_[i]);
		p[0] = pp.x;
		p[1] = pp.y;
		p[2] = pp.z;
		p[3] = 1.0;
		f.push_back(p);
	}
}

void Correspondances::getTransformedFeatures(std::vector<Eigen::Vector4d> &f) {
	for (int i=0; i<src_feat_.size(); i++) {
		Eigen::Vector4d p;
		const pcl::PointXYZ &pp = src_cloud_->at(src_feat_[i]);
		p[0] = pp.x;
		p[1] = pp.y;
		p[2] = pp.z;
		p[3] = 1.0;
		f.push_back(transform_ * p);
	}
}

void Correspondances::getTransformedFeatures2D(std::vector<Eigen::Vector2i> &f) {
	for (int i=0; i<src_feat_.size(); i++) {
		Eigen::Vector4d p;
		const pcl::PointXYZ &pp = src_cloud_->at(src_feat_[i]);
		p[0] = pp.x;
		p[1] = pp.y;
		p[2] = pp.z;
		p[3] = 1.0;
		f.push_back(src_->point(transform_ * p));
	}
}

double Correspondances::findBestSubset(Eigen::Matrix4d &tr, int K, int N) {
	double score = 10000.0f;
	vector<int> best;
	Eigen::Matrix4d bestT;

	std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
	std::uniform_int_distribution<int> distribution(0,src_feat_.size()-1);
	//int dice_roll = distribution(generator);
	auto dice = std::bind ( distribution, rng );

	vector<int> sidx(K);
	vector<int> tidx(K);
	//for (int i = 0; i < K; i++) { idx[i] = i; }

	// 1. Build complete point clouds using filtered and smoothed depth maps
	// 2. Build a full index map of x,y to point cloud index.
	// 3. Generate random subsets of features using index map
	// 4. Find minimum

	for (int n=0; n<N; n++) {

		sidx.clear();
		tidx.clear();

		vector<int> idx;
		for (int k=0; k<K; k++) {
			int r = dice();
			idx.push_back(r);
			sidx.push_back(src_feat_[r]);
			tidx.push_back(targ_feat_[r]);
		}

		Eigen::Matrix4d T;
		float scoreT = estimateTransform(T, sidx, tidx);

		if (scoreT < score) {
			score = scoreT;
			bestT = T;
			best = idx;
		}
	}

	return icp(bestT, tr, best);

	// TODO Add these best points to actual clouds.
	//log_ = best;
	//tr = bestT;
	//uptodate_ = true;
	//return score;
}

Eigen::Matrix4d Correspondances::transform() {
	if (!uptodate_) estimateTransform(transform_);
	return (parent_) ? parent_->transform() * transform_ : transform_;
}
