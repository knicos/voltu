#include <voltu/types/intrinsics.hpp>
#include <Eigen/Eigen>

Eigen::Matrix3d voltu::Intrinsics::matrix() {
	Eigen::Matrix3d K;
	K <<		focal_x,			0.0,	-principle_x,
					0.0,		focal_y,	-principle_y,
					0.0,			0.0,			 1.0;
	return K;
}

Eigen::Vector2i voltu::Intrinsics::size() {
	return { width, height };
}
