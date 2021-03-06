#include <ftl/operators/clipping.hpp>
#include <ftl/cuda/points.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include <ftl/codecs/shapes.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::operators::ClipScene;
using ftl::codecs::Channel;
using ftl::rgbd::Format;

ClipScene::ClipScene(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

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
	clip.origin = MatrixConversion::toCUDA(t.matrix() * r.matrix());
	clip.size = make_float3(width, height, depth);

	ftl::codecs::Shape3D shape;
	shape.id = 0;
	shape.label = "Clipping";
	shape.pose = t.matrix() * r.matrix();
	shape.size = Eigen::Vector3f(width, height, depth);
	shape.type = ftl::codecs::Shape3DType::CLIPPING;

	bool no_clip = config()->value("no_clip", false);
	bool clip_colour = config()->value("clip_colour", false);

	std::list<ftl::codecs::Shape3D> shapes;
	if (in.hasChannel(Channel::Shapes3D)) {
		shapes = in.get<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);
	}
	shapes.push_back(shape);
	in.create<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D).list = shapes;
		
	for (size_t i=0; i<in.frames.size(); ++i) {	
		if (!in.hasFrame(i)) continue;
		auto &f = in.frames[i].cast<ftl::rgbd::Frame>();
		//auto *s = in.sources[i];

		if (f.hasChannel(Channel::Depth)) {
			auto pose = MatrixConversion::toCUDA(f.getPose().cast<float>());

			auto sclip = clip;
			sclip.origin = sclip.origin.getInverse() * pose;
			if (!no_clip) {
				if (clip_colour && f.hasChannel(Channel::Colour)) {
					f.set<ftl::rgbd::VideoFrame>(Channel::Colour);
					f.set<ftl::rgbd::VideoFrame>(Channel::Depth);
					ftl::cuda::clipping(f.createTexture<float>(Channel::Depth), f.getTexture<uchar4>(Channel::Colour), f.getLeftCamera(), sclip, stream);
				} else {
					f.set<ftl::rgbd::VideoFrame>(Channel::Depth);
					ftl::cuda::clipping(f.createTexture<float>(Channel::Depth), f.getLeftCamera(), sclip, stream);
				}
			}
		}
	}

	return true;
}
