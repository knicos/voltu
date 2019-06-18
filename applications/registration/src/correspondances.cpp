#include "correspondances.hpp"
#include <nlohmann/json.hpp>
#include <random>
#include <chrono>

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

bool Correspondances::add(int tx, int ty, int sx, int sy) {
	Eigen::Vector4f p1 = targ_->point(tx,ty);
	Eigen::Vector4f p2 = src_->point(sx,sy);
	PointXYZ pcl_p1;
	pcl_p1.x = p1[0];
	pcl_p1.y = p1[1];
	pcl_p1.z = p1[2];

	PointXYZ pcl_p2;
	pcl_p2.x = p2[0];
	pcl_p2.y = p2[1];
	pcl_p2.z = p2[2];

	if (pcl_p2.z >= 40.0f || pcl_p1.z >= 40.0f) {
		LOG(WARNING) << "Bad point choosen";
		return false;
	}

	targ_cloud_->push_back(pcl_p1);
	src_cloud_->push_back(pcl_p2);
	log_.push_back(make_tuple(tx,ty,sx,sy));

	uptodate_ = false;
	return true;
}

PointXYZ makePCL(Source *s, int x, int y) {
	Eigen::Vector4f p1 = s->point(x,y);
	PointXYZ pcl_p1;
	pcl_p1.x = p1[0];
	pcl_p1.y = p1[1];
	pcl_p1.z = p1[2];
	return pcl_p1;
}

void Correspondances::removeLast() {
	uptodate_ = false;
	targ_cloud_->erase(targ_cloud_->end()-1);
	src_cloud_->erase(src_cloud_->end()-1);
	log_.pop_back();
}

double Correspondances::estimateTransform() {
	uptodate_ = true;
	int n_points = targ_cloud_->size();

	vector<int> idx(n_points);
	for (int i = 0; i < n_points; i++) { idx[i] = i; }

	pcl::registration::TransformationValidationEuclidean<PointXYZ, PointXYZ> validate;
	pcl::registration::TransformationEstimationSVD<PointXYZ,PointXYZ> svd;

	svd.estimateRigidTransformation(*src_cloud_, idx, *targ_cloud_, idx, transform_);

	return validate.validateTransformation(src_cloud_, targ_cloud_, transform_);
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

double Correspondances::findBest(Eigen::Matrix4f &tr, const std::vector<std::tuple<int,int,int,int>> &p, const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &pp, int K, int N) {
	double score = 10000.0f;
	vector<tuple<int,int,int,int>> best;
	Eigen::Matrix4f bestT;

	pcl::registration::TransformationValidationEuclidean<PointXYZ, PointXYZ> validate;
	pcl::registration::TransformationEstimationSVD<PointXYZ,PointXYZ> svd;

	std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
	std::uniform_int_distribution<int> distribution(0,p.size()-1);
	//int dice_roll = distribution(generator);
	auto dice = std::bind ( distribution, rng );

	vector<int> idx(K);
	for (int i = 0; i < K; i++) { idx[i] = i; }

	for (int n=0; n<N; n++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t(new PointCloud<PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new PointCloud<PointXYZ>);

		vector<tuple<int,int,int,int>> ps;

		for (int k=0; k<K; k++) {
			int r = dice();
			//LOG(INFO) << "DICE " << r << "  -  " << K;
			//ps.push_back(p[r]);
			auto [sx,sy,tx,ty] = p[r];
			//LOG(INFO) << "POINT " << sx << "," << sy;
			//auto p1 = makePCL(src_, sx, sy);
			//auto p2 = makePCL(targ_, tx, ty);

			auto [p1,p2] = pp[r];

			if (p1.z >= 30.0f || p2.z >= 30.0f) { k--; continue; }
			ps.push_back(std::make_tuple(tx,ty,sx,sy));
			cloud_s->push_back(p1);
			cloud_t->push_back(p2);
		}

		Eigen::Matrix4f T;
		svd.estimateRigidTransformation(*cloud_s, idx, *cloud_t, idx, T);
		float scoreT = validate.validateTransformation(cloud_s, cloud_t, T);

		if (scoreT < score) {
			score = scoreT;
			best = ps;
			bestT = T;
		}
	}

	// TODO Add these best points to actual clouds.
	log_ = best;
	tr = bestT;
	//uptodate_ = true;
	return score;
}

Eigen::Matrix4f Correspondances::transform() {
	if (!uptodate_) estimateTransform();
	return (parent_) ? parent_->transform() * transform_ : transform_;
}
