#include <ftl/render/overlay.hpp>
#include <ftl/utility/matrix_conversion.hpp>
#include <ftl/cuda_common.hpp>

#include <opencv2/imgproc.hpp>

#include <ftl/codecs/shapes.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::overlay::Overlay;
using ftl::codecs::Channel;
using ftl::overlay::Shape;

namespace {
	constexpr char const *const overlayVertexShader =
		R"(#version 330
		in vec3 vertex;
		uniform float focal;
		uniform float width;
		uniform float height;
		uniform float far;
		uniform float near;
		uniform mat4 pose;
		uniform vec3 scale;

		void main() {
			vec4 vert = pose*(vec4(scale*vertex,1.0));
			vert = vert / vert.w;
			//vec4 pos = vec4(-vert.x*focal / -vert.z / (width/2.0),
			//	vert.y*focal / -vert.z / (height/2.0),
			//	(vert.z-near) / (far-near) * 2.0 - 1.0, 1.0);

			vec4 pos = vec4(
				vert.x*focal / (width/2.0),
				-vert.y*focal / (height/2.0),
				-vert.z * ((far+near) / (far-near)) + (2.0 * near * far / (far-near)),
				//((vert.z - near) / (far - near) * 2.0 - 1.0) * vert.z,
				vert.z
			);

			gl_Position = pos;
		})";

	constexpr char const *const overlayFragmentShader =
		R"(#version 330
		uniform vec4 blockColour;
		out vec4 color;

		void main() {
			color = blockColour;
		})";
}

Overlay::Overlay(nlohmann::json &config) : ftl::Configurable(config) {
	init_ = false;
}

Overlay::~Overlay() {

}

void Overlay::_createShapes() {
	shape_verts_ = {
		// Box
		{-1.0, -1.0, -1.0},
		{1.0, -1.0, -1.0},
		{1.0, 1.0, -1.0},
		{-1.0, 1.0, -1.0},
		{-1.0, -1.0, 1.0},
		{1.0, -1.0, 1.0},
		{1.0, 1.0, 1.0},
		{-1.0, 1.0, 1.0},

		// Camera
		{0.0, 0.0, 0.0},        // 8
		{0.5, 0.28, 0.5},
		{0.5, -0.28, 0.5},
		{-0.5, 0.28, 0.5},
		{-0.5, -0.28, 0.5},

		// Axis lines
		{1.0, 0.0, 0.0},     // 13
		{0.0, -1.0, 0.0},
		{0.0, 0.0, 1.0},

		// Plane XZ big
		{-10.0, 0.0, -10.0},  // 16
		{10.0, 0.0, -10.0},
		{10.0, 0.0, 10.0},
		{-10.0, 0.0, 10.0}
	};

	// Generate a big plane
	for (int x=-9; x<=9; ++x) {
		shape_verts_.push_back({float(x), 0.0, -10.0});
		shape_verts_.push_back({float(x), 0.0, 10.0});
		shape_verts_.push_back({-10.0, 0.0, float(x)});
		shape_verts_.push_back({10.0, 0.0, float(x)});
	}

	shape_tri_indices_ = {
		// Box
		0, 1, 2,
		0, 2, 3,
		1, 5, 6,
		1, 6, 2,
		0, 4, 7,
		0, 7, 3,
		3, 2, 6,
		3, 6, 7,
		0, 1, 5,
		0, 5, 4,

		// Box Lines
		0, 1,       // 30
		1, 5,
		5, 6,
		6, 2,
		2, 1,
		2, 3,
		3, 0,
		3, 7,
		7, 4,
		4, 5,
		6, 7,
		0, 4,

		// Camera
		8, 9, 10,      // 54
		8, 11, 12,
		8, 9, 11,
		8, 10, 12,

		// Camera Lines
		8, 9,           // 66
		8, 10,
		8, 11,
		8, 12,
		9, 10,
		11, 12,
		9, 11,
		10, 12,

		// Axis lines
		8, 13,          // 82
		8, 14,
		8, 15,

		// Big XZ Plane
		16, 17, 18,     // 88
		18, 19, 16
	};

	int i = 20;
	for (int x=-10; x<=10; ++x) {
		shape_tri_indices_.push_back(i++);
		shape_tri_indices_.push_back(i++);
		shape_tri_indices_.push_back(i++);
		shape_tri_indices_.push_back(i++);
	}

	shapes_[Shape::BOX] = {0,30, 30, 12*2};
	shapes_[Shape::CAMERA] = {54, 4*3, 66, 8*2};
	shapes_[Shape::XZPLANE] = {88, 2*3, 94, 40*2};
	shapes_[Shape::AXIS] = {0, 0, 82, 2*3};

	oShader.uploadAttrib("vertex", 3*shape_verts_.size(), 3, sizeof(float), GL_FLOAT, false, shape_verts_.data());
	oShader.uploadAttrib ("indices", 1*shape_tri_indices_.size(), 1, sizeof(int), GL_UNSIGNED_INT, true, shape_tri_indices_.data());
}

void Overlay::_drawFilledShape(Shape shape, const Eigen::Matrix4d &pose, float scale, uchar4 c) {
	if (shapes_.find(shape) ==shapes_.end()) {
		return;
	}

	Eigen::Matrix4f mv = pose.cast<float>();

	auto [offset,count, loffset, lcount] = shapes_[shape];
	UNUSED(loffset);
	UNUSED(lcount);
	oShader.setUniform("scale", scale);
	oShader.setUniform("pose", mv);
	oShader.setUniform("blockColour", Eigen::Vector4f(float(c.x)/255.0f,float(c.y)/255.0f,float(c.z)/255.0f,float(c.w)/255.0f));
	//oShader.drawIndexed(GL_TRIANGLES, offset, count);
	glDrawElements(GL_TRIANGLES, (GLsizei) count, GL_UNSIGNED_INT,
				   (const void *)(offset * sizeof(uint32_t)));
}

void Overlay::_drawOutlinedShape(Shape shape, const Eigen::Matrix4d &pose, const Eigen::Vector3f &scale, uchar4 fill, uchar4 outline) {
	if (shapes_.find(shape) ==shapes_.end()) {
		return;
	}

	Eigen::Matrix4f mv = pose.cast<float>();

	auto [offset,count,loffset,lcount] = shapes_[shape];
	oShader.setUniform("scale", scale);
	oShader.setUniform("pose", mv);

	if (count > 0) {
		oShader.setUniform("blockColour", Eigen::Vector4f(float(fill.x)/255.0f,float(fill.y)/255.0f,float(fill.z)/255.0f,float(fill.w)/255.0f));
		//oShader.drawIndexed(GL_TRIANGLES, offset, count);
		glDrawElements(GL_TRIANGLES, (GLsizei) count, GL_UNSIGNED_INT,
					(const void *)(offset * sizeof(uint32_t)));
	}

	if (lcount != 0) {
		oShader.setUniform("blockColour", Eigen::Vector4f(float(outline.x)/255.0f,float(outline.y)/255.0f,float(outline.z)/255.0f,float(outline.w)/255.0f));
		//oShader.drawIndexed(GL_LINE_LOOP, offset, count);
		glDrawElements(GL_LINES, (GLsizei) lcount, GL_UNSIGNED_INT,
					(const void *)(loffset * sizeof(uint32_t)));
	}
}

void Overlay::_drawAxis(const Eigen::Matrix4d &pose, const Eigen::Vector3f &scale) {
	Eigen::Matrix4f mv = pose.cast<float>();

	auto [offset,count,loffset,lcount] = shapes_[Shape::AXIS];
	UNUSED(offset);
	UNUSED(count);
	UNUSED(lcount);
	oShader.setUniform("scale", scale);
	oShader.setUniform("pose", mv);

	oShader.setUniform("blockColour", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
	//oShader.drawIndexed(GL_LINE_LOOP, offset, count);
	glDrawElements(GL_LINES, (GLsizei) 2, GL_UNSIGNED_INT,
				(const void *)(loffset * sizeof(uint32_t)));

	loffset += 2;
	oShader.setUniform("blockColour", Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
	//oShader.drawIndexed(GL_LINE_LOOP, offset, count);
	glDrawElements(GL_LINES, (GLsizei) 2, GL_UNSIGNED_INT,
				(const void *)(loffset * sizeof(uint32_t)));

	loffset += 2;
	oShader.setUniform("blockColour", Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f));
	//oShader.drawIndexed(GL_LINE_LOOP, offset, count);
	glDrawElements(GL_LINES, (GLsizei) 2, GL_UNSIGNED_INT,
				(const void *)(loffset * sizeof(uint32_t)));
}

void Overlay::draw(NVGcontext *ctx, ftl::data::FrameSet &fs, ftl::rgbd::Frame &frame, const Eigen::Vector2f &screenSize) {
	if (!value("enabled", false)) return;

	double zfar = 8.0f;
	auto intrin = frame.getLeft();
	intrin = intrin.scaled(screenSize[0], screenSize[1]);

	if (!init_) {
		oShader.init("OverlayShader", overlayVertexShader, overlayFragmentShader);
		oShader.bind();
		_createShapes();
		init_ = true;
	} else {
		oShader.bind();
	}

	float3 tris[] = {
		{0.5f, -0.7f, 2.0f},
		{0.2f, -0.5f, 2.0f},
		{0.8f, -0.4f, 2.0f}
	};

	auto pose = MatrixConversion::toCUDA(frame.getPose().cast<float>().inverse());

	tris[0] = pose * tris[0];
	tris[1] = pose * tris[1];
	tris[2] = pose * tris[2];

	glFlush();

	glDepthMask(GL_FALSE);
	glEnable( GL_BLEND );
	glBlendEquation( GL_FUNC_ADD );
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LINE_SMOOTH);

	oShader.setUniform("focal", intrin.fx);
	oShader.setUniform("width", float(intrin.width));
	oShader.setUniform("height", float(intrin.height));
	oShader.setUniform("far", zfar);
	oShader.setUniform("near", 0.1f);  // TODO: but make sure CUDA depth is also normalised like this

	/*oShader.setUniform("blockColour", Eigen::Vector4f(1.0f,1.0f,0.0f,0.5f));
	oShader.uploadAttrib("vertex", sizeof(tris), 3, sizeof(float), GL_FLOAT, false, tris);
	oShader.drawArray(GL_TRIANGLES, 0, 3);

	oShader.setUniform("blockColour", Eigen::Vector4f(1.0f,1.0f,0.0f,1.0f));
	//oShader.uploadAttrib("vertex", sizeof(tris), 3, sizeof(float), GL_FLOAT, false, tris);
	oShader.drawArray(GL_LINE_LOOP, 0, 3);*/

	//glFinish();

	if (value("show_poses", true)) {
		for (size_t i=0; i<fs.frames.size(); ++i) {
			auto &f = fs.frames[i].cast<ftl::rgbd::Frame>();
			if (f.id().id == frame.id().id) continue;

			auto pose = f.getPose(); //.inverse() * state.getPose();

			std::string name = fs.frames[0].name();
	
			auto tpose = frame.getPose().inverse() * pose;
			_drawOutlinedShape(Shape::CAMERA, tpose, Eigen::Vector3f(0.2f,0.2f,0.2f), make_uchar4(255,0,0,80), make_uchar4(255,0,0,255));
			_drawAxis(tpose, Eigen::Vector3f(0.2f, 0.2f, 0.2f));

			float3 textpos;
			textpos.x = tpose(0,3);
			textpos.y = tpose(1,3);
			textpos.z = tpose(2,3);

			float2 textscreen = f.getLeft().camToScreen<float2>(textpos);
			if (textpos.z > 0.1f) nvgText(ctx, textscreen.x, textscreen.y, name.c_str(), nullptr);

			//ftl::overlay::drawCamera(state.getLeft(), out, over_depth_, fs.frames[i].getLeftCamera(), pose, cv::Scalar(0,0,255,255), 0.2,value("show_frustrum", false));
			//if (name) ftl::overlay::drawText(state.getLeft(), out, over_depth_, *name, pos, 0.5, cv::Scalar(0,0,255,255));
		}
	}

	if (value("show_xz_plane", false)) {
		float gscale = value("grid_scale",0.5f);
		_drawOutlinedShape(Shape::XZPLANE, frame.getPose().inverse(), Eigen::Vector3f(gscale,gscale,gscale), make_uchar4(200,200,200,50), make_uchar4(255,255,255,100));
	}

	if (value("show_axis", true)) {
		_drawAxis(frame.getPose().inverse(), Eigen::Vector3f(0.5f, 0.5f, 0.5f));
	}

	if (value("show_shapes", true)) {
		/*if (fs.hasChannel(Channel::Shapes3D)) {
			std::vector<ftl::codecs::Shape3D> shapes;
			fs.get(Channel::Shapes3D, shapes);

			for (auto &s : shapes) {
				auto pose = s.pose.cast<double>();
				//Eigen::Vector4d pos = pose.inverse() * Eigen::Vector4d(0,0,0,1);
				//pos /= pos[3];

				Eigen::Vector3f scale(s.size[0]/2.0f, s.size[1]/2.0f, s.size[2]/2.0f);

				if (s.type == ftl::codecs::Shape3DType::CAMERA) {
						//auto pose = s.pose;
					auto name = s.label;
					_drawOutlinedShape(Shape::CAMERA, state.getPose().inverse() * pose, Eigen::Vector3f(0.2f,0.2f,0.2f), make_uchar4(255,0,0,80), make_uchar4(255,0,0,255));
					_drawAxis(state.getPose().inverse() * pose, Eigen::Vector3f(0.2f, 0.2f, 0.2f));
				} else {
					_drawOutlinedShape(Shape::BOX, state.getPose().inverse() * pose, scale, make_uchar4(255,0,255,80), make_uchar4(255,0,255,255));
				}

				//ftl::overlay::drawFilledBox(state.getLeft(), out, over_depth_, pose, cv::Scalar(0,0,255,50), s.size.cast<double>());
				//ftl::overlay::drawBox(state.getLeft(), out, over_depth_, pose, cv::Scalar(0,0,255,255), s.size.cast<double>());
				//ftl::overlay::drawText(state.getLeft(), out, over_depth_, s.label, pos, 0.5, cv::Scalar(0,0,255,100));
			}
		}*/

		for (size_t i=0; i<fs.frames.size(); ++i) {
			if (fs.frames[i].hasChannel(Channel::Shapes3D)) {
				const auto &shapes = fs.frames[i].get<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);

				for (auto &s : shapes) {
					auto pose = s.pose.cast<double>(); //.inverse() * state.getPose();
					//Eigen::Vector4d pos = pose.inverse() * Eigen::Vector4d(0,0,0,1);
					//pos /= pos[3];

					Eigen::Vector3f scale(s.size[0]/2.0f, s.size[1]/2.0f, s.size[2]/2.0f);

					auto tpose = frame.getPose().inverse() * pose;

					switch (s.type) {
					case ftl::codecs::Shape3DType::CAMERA: _drawOutlinedShape(Shape::CAMERA, tpose, scale, make_uchar4(255,0,0,80), make_uchar4(255,0,0,255)); break;
					case ftl::codecs::Shape3DType::CLIPPING: _drawOutlinedShape(Shape::BOX, tpose, scale, make_uchar4(255,0,255,80), make_uchar4(255,0,255,255)); break;
					case ftl::codecs::Shape3DType::ARUCO: _drawAxis(tpose, Eigen::Vector3f(0.2f, 0.2f, 0.2f)); break;
					default: break;
					}

					if (s.label.size() > 0) {
						float3 textpos;
						textpos.x = tpose(0,3);
						textpos.y = tpose(1,3);
						textpos.z = tpose(2,3);

						float2 textscreen = frame.getLeft().camToScreen<float2>(textpos);
						if (textpos.z > 0.1f) nvgText(ctx, textscreen.x, textscreen.y, s.label.c_str(), nullptr);
					}

					//ftl::overlay::drawBox(state.getLeft(), out, over_depth_, pose, cv::Scalar(0,0,255,100), s.size.cast<double>());
					//ftl::overlay::drawText(state.getLeft(), out, over_depth_, s.label, pos, 0.5, cv::Scalar(0,0,255,100));
				}
			}
		}
	}

	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_BLEND);

	//cv::flip(out, out, 0);
}

/*void ftl::overlay::draw3DLine(
		const ftl::rgbd::Camera &cam,
		cv::Mat &colour,
		cv::Mat &depth,
		const Eigen::Vector4d &begin,
		const Eigen::Vector4d &end,
		const cv::Scalar &linecolour) {


	auto begin_pos = cam.camToScreen<int2>(make_float3(begin[0], begin[1], begin[2]));
	auto end_pos = cam.camToScreen<int2>(make_float3(end[0], end[1], end[2]));

	cv::LineIterator lineit(colour, cv::Point(begin_pos.x, begin_pos.y), cv::Point(end_pos.x, end_pos.y));
	double z_grad = (end[2] - begin[2]) / lineit.count;
	double current_z = begin[2];

	for(int i = 0; i < lineit.count; i++, ++lineit) {
		colour.at<cv::Vec4b>(lineit.pos()) = linecolour;
		depth.at<float>(lineit.pos()) = current_z;
		current_z += z_grad;
	}
}

void ftl::overlay::drawPoseBox(
		const ftl::rgbd::Camera &cam,
		cv::Mat &colour,
		cv::Mat &depth,
		const Eigen::Matrix4d &pose,
		const cv::Scalar &linecolour,
		double size) {

	double size2 = size/2.0;

	Eigen::Vector4d p001 = pose.inverse() * Eigen::Vector4d(size2,size2,-size2,1);
	Eigen::Vector4d p011 = pose.inverse() * Eigen::Vector4d(size2,-size2,-size2,1);
	Eigen::Vector4d p111 = pose.inverse() * Eigen::Vector4d(-size2,-size2,-size2,1);
	Eigen::Vector4d p101 = pose.inverse() * Eigen::Vector4d(-size2,size2,-size2,1);
	Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(-size2,-size2,size2,1);
	Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(-size2,size2,size2,1);
	Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(size2,-size2,size2,1);
	Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(size2,size2,size2,1);

	p001 /= p001[3];
	p011 /= p011[3];
	p111 /= p111[3];
	p101 /= p101[3];
	p110 /= p110[3];
	p100 /= p100[3];
	p010 /= p010[3];
	p000 /= p000[3];

	if (p001[2] < 0.1 || p011[2] < 0.1 || p111[2] < 0.1 || p101[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

	draw3DLine(cam, colour, depth, p000, p001, linecolour);
	draw3DLine(cam, colour, depth, p000, p010, linecolour);
	draw3DLine(cam, colour, depth, p000, p100, linecolour);

	draw3DLine(cam, colour, depth, p001, p011, linecolour);
	draw3DLine(cam, colour, depth, p001, p101, linecolour);

	draw3DLine(cam, colour, depth, p010, p011, linecolour);
	draw3DLine(cam, colour, depth, p010, p110, linecolour);

	draw3DLine(cam, colour, depth, p100, p101, linecolour);
	draw3DLine(cam, colour, depth, p100, p110, linecolour);

	draw3DLine(cam, colour, depth, p101, p111, linecolour);
	draw3DLine(cam, colour, depth, p110, p111, linecolour);
	draw3DLine(cam, colour, depth, p011, p111, linecolour);
}

void ftl::overlay::drawBox(
		const ftl::rgbd::Camera &cam,
		cv::Mat &colour,
		cv::Mat &depth,
		const Eigen::Matrix4d &pose,
		const cv::Scalar &linecolour,
		const Eigen::Vector3d &size) {

	double size2x = size[0]/2.0;
	double size2y = size[1]/2.0;
	double size2z = size[2]/2.0;

	Eigen::Vector4d p001 = pose.inverse() * Eigen::Vector4d(size2x,size2y,-size2z,1);
	Eigen::Vector4d p011 = pose.inverse() * Eigen::Vector4d(size2x,-size2y,-size2z,1);
	Eigen::Vector4d p111 = pose.inverse() * Eigen::Vector4d(-size2x,-size2y,-size2z,1);
	Eigen::Vector4d p101 = pose.inverse() * Eigen::Vector4d(-size2x,size2y,-size2z,1);
	Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(-size2x,-size2y,size2z,1);
	Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(-size2x,size2y,size2z,1);
	Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(size2x,-size2y,size2z,1);
	Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(size2x,size2y,size2z,1);

	p001 /= p001[3];
	p011 /= p011[3];
	p111 /= p111[3];
	p101 /= p101[3];
	p110 /= p110[3];
	p100 /= p100[3];
	p010 /= p010[3];
	p000 /= p000[3];

	if (p001[2] < 0.1 || p011[2] < 0.1 || p111[2] < 0.1 || p101[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

	draw3DLine(cam, colour, depth, p000, p001, linecolour);
	draw3DLine(cam, colour, depth, p000, p010, linecolour);
	draw3DLine(cam, colour, depth, p000, p100, linecolour);

	draw3DLine(cam, colour, depth, p001, p011, linecolour);
	draw3DLine(cam, colour, depth, p001, p101, linecolour);

	draw3DLine(cam, colour, depth, p010, p011, linecolour);
	draw3DLine(cam, colour, depth, p010, p110, linecolour);

	draw3DLine(cam, colour, depth, p100, p101, linecolour);
	draw3DLine(cam, colour, depth, p100, p110, linecolour);

	draw3DLine(cam, colour, depth, p101, p111, linecolour);
	draw3DLine(cam, colour, depth, p110, p111, linecolour);
	draw3DLine(cam, colour, depth, p011, p111, linecolour);
}

void ftl::overlay::drawFilledBox(
		const ftl::rgbd::Camera &cam,
		cv::Mat &colour,
		cv::Mat &depth,
		const Eigen::Matrix4d &pose,
		const cv::Scalar &linecolour,
		const Eigen::Vector3d &size) {

	double size2x = size[0]/2.0;
	double size2y = size[1]/2.0;
	double size2z = size[2]/2.0;

	Eigen::Vector4d p001 = pose.inverse() * Eigen::Vector4d(size2x,size2y,-size2z,1);
	Eigen::Vector4d p011 = pose.inverse() * Eigen::Vector4d(size2x,-size2y,-size2z,1);
	Eigen::Vector4d p111 = pose.inverse() * Eigen::Vector4d(-size2x,-size2y,-size2z,1);
	Eigen::Vector4d p101 = pose.inverse() * Eigen::Vector4d(-size2x,size2y,-size2z,1);
	Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(-size2x,-size2y,size2z,1);
	Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(-size2x,size2y,size2z,1);
	Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(size2x,-size2y,size2z,1);
	Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(size2x,size2y,size2z,1);

	p001 /= p001[3];
	p011 /= p011[3];
	p111 /= p111[3];
	p101 /= p101[3];
	p110 /= p110[3];
	p100 /= p100[3];
	p010 /= p010[3];
	p000 /= p000[3];

	if (p001[2] < 0.1 || p011[2] < 0.1 || p111[2] < 0.1 || p101[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

	std::array<cv::Point, 4> pts;

	auto p = cam.camToScreen<int2>(make_float3(p000[0], p000[1], p000[2]));
	pts[0] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p001[0], p001[1], p001[2]));
	pts[1] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p011[0], p011[1], p011[2]));
	pts[2] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p010[0], p010[1], p010[2]));
	pts[3] = cv::Point(p.x, p.y);
	cv::fillConvexPoly(colour, pts, linecolour);

	p = cam.camToScreen<int2>(make_float3(p100[0], p100[1], p100[2]));
	pts[0] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p101[0], p101[1], p101[2]));
	pts[1] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p111[0], p111[1], p111[2]));
	pts[2] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p110[0], p110[1], p110[2]));
	pts[3] = cv::Point(p.x, p.y);
	cv::fillConvexPoly(colour, pts, linecolour);

	p = cam.camToScreen<int2>(make_float3(p000[0], p000[1], p000[2]));
	pts[0] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p010[0], p010[1], p010[2]));
	pts[1] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p110[0], p110[1], p110[2]));
	pts[2] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p100[0], p100[1], p100[2]));
	pts[3] = cv::Point(p.x, p.y);
	cv::fillConvexPoly(colour, pts, linecolour);

	p = cam.camToScreen<int2>(make_float3(p001[0], p001[1], p001[2]));
	pts[0] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p011[0], p011[1], p011[2]));
	pts[1] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p111[0], p111[1], p111[2]));
	pts[2] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p101[0], p101[1], p101[2]));
	pts[3] = cv::Point(p.x, p.y);
	cv::fillConvexPoly(colour, pts, linecolour);

	p = cam.camToScreen<int2>(make_float3(p000[0], p000[1], p000[2]));
	pts[0] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p001[0], p001[1], p001[2]));
	pts[1] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p101[0], p101[1], p101[2]));
	pts[2] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p100[0], p100[1], p100[2]));
	pts[3] = cv::Point(p.x, p.y);
	cv::fillConvexPoly(colour, pts, linecolour);

	p = cam.camToScreen<int2>(make_float3(p010[0], p010[1], p010[2]));
	pts[0] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p011[0], p011[1], p011[2]));
	pts[1] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p111[0], p111[1], p111[2]));
	pts[2] = cv::Point(p.x, p.y);
	p = cam.camToScreen<int2>(make_float3(p110[0], p110[1], p110[2]));
	pts[3] = cv::Point(p.x, p.y);
	cv::fillConvexPoly(colour, pts, linecolour);
}

void ftl::overlay::drawRectangle(
		const ftl::rgbd::Camera &cam,
		cv::Mat &colour,
		cv::Mat &depth,
		const Eigen::Matrix4d &pose,
		const cv::Scalar &linecolour,
		double width, double height) {

	double width2 = width/2.0;
	double height2 = height/2.0;

	Eigen::Vector4d p001 = pose.inverse() * Eigen::Vector4d(width2,height2,0,1);
	Eigen::Vector4d p011 = pose.inverse() * Eigen::Vector4d(width2,-height2,0,1);
	Eigen::Vector4d p111 = pose.inverse() * Eigen::Vector4d(-width2,-height2,0,1);
	Eigen::Vector4d p101 = pose.inverse() * Eigen::Vector4d(-width2,height2,0,1);

	p001 /= p001[3];
	p011 /= p011[3];
	p111 /= p111[3];
	p101 /= p101[3];

	if (p001[2] < 0.1 || p011[2] < 0.1 || p111[2] < 0.1 || p101[2] < 0.1) return;

	draw3DLine(cam, colour, depth, p001, p011, linecolour);
	draw3DLine(cam, colour, depth, p001, p101, linecolour);
	draw3DLine(cam, colour, depth, p101, p111, linecolour);
	draw3DLine(cam, colour, depth, p011, p111, linecolour);
}

void ftl::overlay::drawPoseCone(
		const ftl::rgbd::Camera &cam,
		cv::Mat &colour,
		cv::Mat &depth,
		const Eigen::Matrix4d &pose,
		const cv::Scalar &linecolour,
		double size) {

	double size2 = size;

	Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(-size2,-size2,size2,1);
	Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(-size2,size2,size2,1);
	Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(size2,-size2,size2,1);
	Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(size2,size2,size2,1);
	Eigen::Vector4d origin = pose.inverse() * Eigen::Vector4d(0,0,0,1);

	p110 /= p110[3];
	p100 /= p100[3];
	p010 /= p010[3];
	p000 /= p000[3];
	origin /= origin[3];

	if (origin[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

	draw3DLine(cam, colour, depth, p000, origin, linecolour);
	draw3DLine(cam, colour, depth, p000, p010, linecolour);
	draw3DLine(cam, colour, depth, p000, p100, linecolour);

	draw3DLine(cam, colour, depth, p010, origin, linecolour);
	draw3DLine(cam, colour, depth, p010, p110, linecolour);

	draw3DLine(cam, colour, depth, p100, origin, linecolour);
	draw3DLine(cam, colour, depth, p100, p110, linecolour);

	draw3DLine(cam, colour, depth, p110, origin, linecolour);
}

void ftl::overlay::drawCamera(
		const ftl::rgbd::Camera &vcam,
		cv::Mat &colour,
		cv::Mat &depth,
		const ftl::rgbd::Camera &camera,
		const Eigen::Matrix4d &pose,
		const cv::Scalar &linecolour,
		double scale, bool frustrum) {

	//double size2 = size;

	const auto &params = camera;
	double width = (static_cast<double>(params.width) / static_cast<double>(params.fx)) * scale;
	double height = (static_cast<double>(params.height) / static_cast<double>(params.fx)) * scale;
	double width2 = width / 2.0;
	double height2 = height / 2.0;

	double principx = (((static_cast<double>(params.width) / 2.0) + params.cx) / static_cast<double>(params.fx)) * scale;
	double principy = (((static_cast<double>(params.height) / 2.0) + params.cy) / static_cast<double>(params.fx)) * scale;

	auto ptcoord = params.screenToCam(0,0,scale);
	Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
	ptcoord = params.screenToCam(0,params.height,scale);
	Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
	ptcoord = params.screenToCam(params.width,0,scale);
	Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
	ptcoord = params.screenToCam(params.width,params.height,scale);
	Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
	Eigen::Vector4d origin = pose.inverse() * Eigen::Vector4d(0,0,0,1);

	p110 /= p110[3];
	p100 /= p100[3];
	p010 /= p010[3];
	p000 /= p000[3];
	origin /= origin[3];

	if (origin[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

	draw3DLine(vcam, colour, depth, p000, origin, linecolour);
	draw3DLine(vcam, colour, depth, p000, p010, linecolour);
	draw3DLine(vcam, colour, depth, p000, p100, linecolour);

	draw3DLine(vcam, colour, depth, p010, origin, linecolour);
	draw3DLine(vcam, colour, depth, p010, p110, linecolour);

	draw3DLine(vcam, colour, depth, p100, origin, linecolour);
	draw3DLine(vcam, colour, depth, p100, p110, linecolour);

	draw3DLine(vcam, colour, depth, p110, origin, linecolour);

	if (frustrum) {
		const double fscale = 16.0;
		ptcoord = params.screenToCam(0,0,fscale);
		Eigen::Vector4d f110 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
		ptcoord = params.screenToCam(0,params.height,fscale);
		Eigen::Vector4d f100 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
		ptcoord = params.screenToCam(params.width,0,fscale);
		Eigen::Vector4d f010 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
		ptcoord = params.screenToCam(params.width,params.height,fscale);
		Eigen::Vector4d f000 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);

		f110 /= f110[3];
		f100 /= f100[3];
		f010 /= f010[3];
		f000 /= f000[3];

		if (f110[2] < 0.1 || f100[2] < 0.1 || f010[2] < 0.1 || f000[2] < 0.1) return;

		draw3DLine(vcam, colour, depth, f000, p000, cv::Scalar(0,255,0,0));
		draw3DLine(vcam, colour, depth, f010, p010, cv::Scalar(0,255,0,0));
		draw3DLine(vcam, colour, depth, f100, p100, cv::Scalar(0,255,0,0));
		draw3DLine(vcam, colour, depth, f110, p110, cv::Scalar(0,255,0,0));

		draw3DLine(vcam, colour, depth, f000, f010, cv::Scalar(0,255,0,0));
		draw3DLine(vcam, colour, depth, f000, f100, cv::Scalar(0,255,0,0));
		draw3DLine(vcam, colour, depth, f010, f110, cv::Scalar(0,255,0,0));
		draw3DLine(vcam, colour, depth, f100, f110, cv::Scalar(0,255,0,0));
	}
}

void ftl::overlay::drawText(
		const ftl::rgbd::Camera &cam,
		cv::Mat &colour,
		cv::Mat &depth,
		const std::string &text,
		const Eigen::Vector4d &pos,
		double size,
		const cv::Scalar &textcolour) {

	auto pt = cam.camToScreen<int2>(make_float3(pos[0], pos[1], pos[2]));
	if (pos[2] < 0.1) return;
	cv::putText(colour, text, cv::Point(pt.x, colour.rows-pt.y), 0, size, textcolour, 1, cv::LINE_8, true);
}*/
