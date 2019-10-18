#include "loguru.hpp"
#include "vr.hpp"

Eigen::Matrix3d getCameraMatrix(const double tanx1,
								const double tanx2,
								const double tany1,
								const double tany2,
								const double size_x,
								const double size_y) {
	
	Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
	
	CHECK(tanx1 < 0 && tanx2 > 0 && tany1 < 0 && tany2 > 0);
	CHECK(size_x > 0 && size_y > 0);

	double fx = size_x / (-tanx1 + tanx2);
	double fy = size_y / (-tany1 + tany2);
	C(0,0) = fx;
	C(1,1) = fy;
	C(0,2) = tanx1 * fx;
	C(1,2) = tany1 * fy;

	// safe to remove
	CHECK((int) (abs(tanx1 * fx) + abs(tanx2 * fx)) == (int) size_x);
	CHECK((int) (abs(tany1 * fy) + abs(tany2 * fy)) == (int) size_y);

	return C;
}

Eigen::Matrix3d getCameraMatrix(vr::IVRSystem *vr, const vr::Hmd_Eye &eye) {
	float tanx1, tanx2, tany1, tany2;
	uint32_t size_x, size_y;
	vr->GetProjectionRaw(eye, &tanx1, &tanx2, &tany1, &tany2);
	vr->GetRecommendedRenderTargetSize(&size_x, &size_y);
	return getCameraMatrix(tanx1, tanx2, tany1, tany2, size_x, size_y);
}