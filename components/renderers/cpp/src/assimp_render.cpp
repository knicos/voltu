#include <ftl/render/assimp_render.hpp>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <GL/gl.h>

using ftl::render::AssimpRenderer;

namespace {
	constexpr char const *const assimpVertexShader =
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

	constexpr char const *const assimpFragmentShader =
		R"(#version 330
		uniform vec4 blockColour;
		out vec4 color;
		
		void main() {
			color = blockColour;
		})";
}

AssimpRenderer::AssimpRenderer(nlohmann::json &config) : ftl::render::Renderer(config) {
	init_ = false;
}

AssimpRenderer::~AssimpRenderer() {

}

void AssimpRenderer::begin(ftl::rgbd::Frame &f, ftl::codecs::Channel c) {
	if (!scene_) {
		throw FTL_Error("No Assimp scene");
	}

	out_ = &f;
	outchan_ = c;

	if (!init_) {
		shader_.init("AssimpShader", assimpVertexShader, assimpFragmentShader);
        shader_.bind();
		init_ = true;
	} else {
	    shader_.bind();
    }

	const auto &intrin = f.getLeftCamera();

	glViewport(0, 0, intrin.width, intrin.height);					// Reset The Current Viewport

	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();							// Reset The Projection Matrix

	//gluPerspective(2.0f*atan(0.5f*float(intrin.height) / intrin.fy),(GLfloat)intrin.width/(GLfloat)intrin.height,intrin.minDepth,intrin.maxDepth);

	glMatrixMode(GL_MODELVIEW);						// Select The Modelview Matrix
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);		 // Enables Smooth Shading
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClearDepth(1.0f);				// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);		// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);			// The Type Of Depth Test To Do

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	scene_->load();
}

void AssimpRenderer::end() {
	glFlush();
}

void AssimpRenderer::render() {

}

void AssimpRenderer::blend(ftl::codecs::Channel) {
	throw FTL_Error("Blend not supported in Assimp render");
}

void AssimpRenderer::setScene(ftl::render::AssimpScene *scene) {
	scene_ = scene;
}
