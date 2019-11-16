#include <ftl/operators/clipping.hpp>
#include <ftl/cuda/points.hpp>
#include <ftl/utility/matrix_conversion.hpp>

using ftl::operators::ClipScene;
using ftl::codecs::Channel;
using ftl::rgbd::Format;

ClipScene::ClipScene(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

ClipScene::~ClipScene() {

}

// TODO: Put in common place
static Eigen::Affine3d create_rotation_matrix(float ax, float ay, float az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * rx * ry;
}

bool ClipScene::apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) {
	auto &c = *config();
	float rx = c.value("pitch", 0.0f);
	float ry = c.value("yaw", 0.0f);
	float rz = c.value("roll", 0.0f);
	float x = c.value("x", 0.0f);
	float y = c.value("y", 0.0f);
	float z = c.value("z", 0.0f);
	float width = c.value("width", 1.0f);
	float height = c.value("height", 1.0f);
	float depth = c.value("depth", 1.0f);

	Eigen::Affine3f r = create_rotation_matrix(rx, ry, rz).cast<float>();
	Eigen::Translation3f trans(Eigen::Vector3f(x,y,z));
	Eigen::Affine3f t(trans);

	ftl::cuda::ClipSpace clip;
	clip.origin = MatrixConversion::toCUDA(r.matrix() * t.matrix());
	clip.size = make_float3(width, height, depth);
		
	for (size_t i=0; i<in.frames.size(); ++i) {	
		auto &f = in.frames[i];
		auto *s = in.sources[i];

		auto pose = MatrixConversion::toCUDA(s->getPose().cast<float>());

		auto sclip = clip;
		sclip.origin = sclip.origin * pose;
		ftl::cuda::clipping(f.createTexture<float>(Channel::Depth), s->parameters(), sclip, stream);
	}

	return true;
}
