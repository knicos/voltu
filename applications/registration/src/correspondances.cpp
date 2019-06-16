#include "correspondances.hpp"
#include <nlohmann/json.hpp>

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

Eigen::Matrix4f Correspondances::transform() {
	if (!uptodate_) estimateTransform();
	return (parent_) ? parent_->transform() * transform_ : transform_;
}
