#include <nanogui/glutil.h>
#include <ftl/render/GLRender.hpp>
#include <ftl/exception.hpp>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <loguru.hpp>

using ftl::render::GLRender;
using ftl::codecs::Channel;

namespace {
	constexpr char const *const rendVertexShader =
		R"(#version 330
		in vec3 vertex;
		uniform mat4 pose;
		uniform sampler2D depthMap;
		uniform float cx;
		uniform float cy;
		uniform float f;
		uniform float width;
		uniform float height;
		out float depths;
		out vec2 uvs;

		void main() {
			vec2 tc = vec2(vertex.x / width, vertex.y / height);
			uvs = tc;
			float d = float(texture(depthMap, tc));
			depths = d;
			float x = ((vertex.x+cx) / f) * d;
			float y = ((vertex.y+cy) / f) * d;
			vec4 v = vec4(x, y, d, 1.0);
			vec4 vert = pose*v;
			gl_Position = vert;
		})";

	constexpr char const *const rendFragmentShader =
		R"(#version 330
		layout(location = 0) out vec4 color;
		uniform sampler2D colourMap;
		in vec2 uv;

		void main() {
			color = vec4(texture(colourMap, uv).rgb,1.0);
		})";

	constexpr char const *const rendGeomShader =
		R"(#version 330
		layout (triangles) in;
		layout (triangle_strip, max_vertices=3) out;
		in float depths[];
		in vec2 uvs[];
		out vec2 uv;

		void main() {
			if (depths[0] > 0 && depths[1] > 0 && depths[2] > 0) {  
				gl_Position = gl_in[0].gl_Position; 
				uv = uvs[0];
				EmitVertex();
				gl_Position = gl_in[1].gl_Position; 
				uv = uvs[1];
				EmitVertex();
				gl_Position = gl_in[2].gl_Position;
				uv = uvs[2]; 
				EmitVertex();
				EndPrimitive();
			}
		})";
}

GLRender::GLRender(nlohmann::json &config) : ftl::render::FSRenderer(config) {
	shader_ = new nanogui::GLShader();
}

GLRender::~GLRender() {
	delete shader_;
}

void GLRender::begin(ftl::rgbd::Frame &out, ftl::codecs::Channel c) {
	out_ = &out;
	out.createTexture<uchar4>(c);
}

void GLRender::end() {
	sets_.clear();
	out_ = nullptr;
}

bool GLRender::submit(ftl::data::FrameSet *in, ftl::codecs::Channels<0> chans, const Eigen::Matrix4d &t) {
	auto &s = sets_.emplace_back();
	s.fs = in;
	s.channels = chans;
	s.transform = t;

	// Generate depth map opengl textures
	/*for (const auto &f : in->frames) {
		auto &tex = textures_[f.id().id];
		tex.copyFrom(f.get<cv::cuda::GpuMat>(Channel::Depth));
	}*/

	return true;
}

void GLRender::render() {
	// Create render target.

	if (sets_.empty() || !out_) return;

	glDisable(GL_STENCIL_TEST);
	glDisable(GL_SCISSOR_TEST);
	glEnable(GL_DEPTH_TEST);

	const auto &outintrin = out_->getLeft();

	if (outTex_ == 0) {
		fbuf_ = 0;
		glGenFramebuffers(1, &fbuf_);
		glBindFramebuffer(GL_FRAMEBUFFER, fbuf_);

		depthbuf_ = 0;
		glGenRenderbuffers(1, &depthbuf_);
		glBindRenderbuffer(GL_RENDERBUFFER, depthbuf_);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, outintrin.width, outintrin.height);

		glGenTextures(1, &outTex_);
		glBindTexture(GL_TEXTURE_2D, outTex_);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, outintrin.width, outintrin.height, 0, GL_BGRA, GL_UNSIGNED_BYTE, nullptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_2D, 0);

		glGenBuffers(1, &outPBO_);
		// Make this the current UNPACK buffer (OpenGL is state-based)
		glBindBuffer(GL_PIXEL_PACK_BUFFER, outPBO_);
		// Allocate data for the buffer. 4-channel 8-bit image or 1-channel float
		glBufferData(GL_PIXEL_PACK_BUFFER, outintrin.width * outintrin.height * 4, NULL, GL_DYNAMIC_COPY);

		cudaSafeCall(cudaGraphicsGLRegisterBuffer(&cuda_res_, outPBO_, cudaGraphicsRegisterFlagsNone)); // cudaGraphicsRegisterFlagsWriteDiscard
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

		//glBindTexture(GL_TEXTURE_2D, outTex_.texture());
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, outTex_, 0);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthbuf_);

		glBindRenderbuffer(GL_RENDERBUFFER, 0);
		glBindTexture(GL_TEXTURE_2D, 0);

		auto err = glGetError();
		if (err != 0) LOG(ERROR) << "OpenGL FB error: " << err;

		//GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
		//glDrawBuffers(1, DrawBuffers);

		if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			throw FTL_Error("Could not create opengl frame buffer");
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		err = glGetError();
		if (err != 0) LOG(ERROR) << "OpenGL FB error: " << err;
		LOG(INFO) << "GL Frame Buffer Created";
	}

	if (fbuf_ == 0) {
		LOG(ERROR) << "No framebuffer";
	}

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(out_->stream());
	/*auto colmati = outTex_.map(out_->stream());
	colmati.setTo(cv::Scalar(255,0,0,255), cvstream);
	outTex_.unmap(out_->stream());
	cudaSafeCall(cudaStreamSynchronize(out_->stream()));*/

	glBindFramebuffer(GL_FRAMEBUFFER, fbuf_);
	glViewport(0,0,outintrin.width,outintrin.height);
	//LOG(INFO) << "Render " << outintrin.width << "x" << outintrin.height << " " << fbuf_;
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//GLfloat clearColor[4] = {255.0f,0.0f,0.0f,255.0f};
	//glClearBufferfv( GL_COLOR, 0, clearColor );

	auto err = glGetError();
	if (err != 0) LOG(ERROR) << "OpenGL error: " << err;

	// Build+upload vertices and indices if not already there.
	if (vertices_.empty()) {
		shader_->init("RenderShader", rendVertexShader, rendFragmentShader, rendGeomShader);
		shader_->bind();
		const auto &f = sets_[0].fs->firstFrame().cast<ftl::rgbd::Frame>();
		const ftl::rgbd::Camera &intrin = f.getLeft();

		int ix = 0;
		int iix = 0;
		vertices_.resize(intrin.width*intrin.height*3);
		indices_.resize(intrin.width*intrin.height*3*4);

		for (int y=0; y<intrin.height; ++y) {
		for (int x=0; x<intrin.width; ++x) {
			//float3 p = intrin.screenToCam(x,y,-1.0f);
			//LOG(INFO) << "POINT " << p.x << "," << p.y << "," << p.z;
			vertices_[ix++] = float(x);
			vertices_[ix++] = float(y);
			vertices_[ix++] = 1.0f;
			//vertices_[ix++] = float(x) / float(intrin.width);
			//vertices_[ix++] = float(y) / float(intrin.height);

			indices_[iix++] = x + y * intrin.width;
			indices_[iix++] = x - 1 + y * intrin.width;
			indices_[iix++] = x + (y+1) * intrin.width;

			indices_[iix++] = x + y * intrin.width;
			indices_[iix++] = x + 1 + y * intrin.width;
			indices_[iix++] = x + (y+1) * intrin.width;

			indices_[iix++] = x + y * intrin.width;
			indices_[iix++] = x - 1 + y * intrin.width;
			indices_[iix++] = x + (y-1) * intrin.width;

			indices_[iix++] = x + y * intrin.width;
			indices_[iix++] = x + 1 + y * intrin.width;
			indices_[iix++] = x + (y-1) * intrin.width;
		}
		}

		shader_->uploadAttrib("vertex", vertices_.size()/3, 3, sizeof(float)*3, GL_FLOAT, false, vertices_.data());
		shader_->uploadAttrib("indices", indices_.size(), 1, sizeof(int), GL_UNSIGNED_INT, true, indices_.data());
	} else {
		shader_->bind();
	}

	float l = outintrin.screenToCam(0u,outintrin.height/2u,0.1f).x;
	float r = outintrin.screenToCam(outintrin.width,outintrin.height/2u,0.1f).x;
	float t = outintrin.screenToCam(outintrin.width/2u,0u,0.1f).y;
	float b = outintrin.screenToCam(outintrin.width/2u,outintrin.height,0.1f).y;
	Eigen::Matrix4f p = nanogui::frustum(l, r, b, t, 0.1f, 12.0f);//.transpose();
	Eigen::Matrix4f v = out_->getPose().cast<float>().inverse();

	for (const auto &s : sets_) {
		for (const auto &f : s.fs->frames) {
			const auto &rgbdf = f.cast<ftl::rgbd::Frame>();
			const auto &intrin = rgbdf.getLeft();

			Eigen::Matrix4f m = rgbdf.getPose().cast<float>();//.inverse();
			m = p * v * m;
			m = m; //.transpose();

			depth_input_.copyFrom(f.get<cv::cuda::GpuMat>(Channel::Depth));
			colour_input_.copyFrom(f.get<cv::cuda::GpuMat>(Channel::Colour));

			// Set pose
			shader_->setUniform("pose", m);
			shader_->setUniform("width", float(intrin.width));
			shader_->setUniform("height", float(intrin.height));
			shader_->setUniform("f", float(intrin.fx));
			shader_->setUniform("cx", float(intrin.cx));
			shader_->setUniform("cy", float(intrin.cy));

			GLuint tex = depth_input_.texture();
			if (tex == 0) LOG(ERROR) << "No depth texture";
			glActiveTexture(GL_TEXTURE0);
    		glBindTexture(GL_TEXTURE_2D, tex);
			shader_->setUniform("depthMap", 0);

			tex = colour_input_.texture();
			if (tex == 0) LOG(ERROR) << "No colour texture";
			glActiveTexture(GL_TEXTURE1);
    		glBindTexture(GL_TEXTURE_2D, tex);
			shader_->setUniform("colourMap", 1);

			// Set the depth map texture
			//glDrawElements(GL_TRIANGLES, (GLsizei) indices_.size(), GL_UNSIGNED_INT, (const void *)(0));
			//shader_.drawArray(GL_POINTS, 0, vertices_.size()/3);
			shader_->drawIndexed(GL_TRIANGLES, 0, indices_.size()/3);

			//LOG(INFO) << "Points drawn: " << vertices_.size()/3;
		}
	}

	//LOG(INFO) << "GL Render done, copying output";

	glBindBuffer(GL_PIXEL_PACK_BUFFER, outPBO_);
	// Select the appropriate texture
	glBindTexture(GL_TEXTURE_2D, outTex_);

	//glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
	glReadPixels(0, 0, outintrin.width, outintrin.height, GL_BGRA, GL_UNSIGNED_BYTE, 0);

	glBindTexture(GL_TEXTURE_2D, 0);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	//glFlush();
	//glFinish();
	err = glGetError();
	if (err != 0) LOG(ERROR) << "OpenGL error: " << err;

	void *devptr;
	size_t size;
	cudaSafeCall(cudaGraphicsMapResources(1, &cuda_res_, out_->stream()));
	cudaSafeCall(cudaGraphicsResourceGetMappedPointer(&devptr, &size, cuda_res_));
	cv::cuda::GpuMat colmat(outintrin.height, outintrin.width, CV_8UC4, devptr, outintrin.width*4);

	//auto colmat = outTex_.map(out_->stream());
	//colmat.setTo(cv::Scalar(255,0,0,255), cvstream);
	//LOG(INFO) << "COLMAT: " << colmat.cols << ", " << colmat.rows << ", " << uint64_t(colmat.data);
	colmat.copyTo(out_->get<cv::cuda::GpuMat>(Channel::Colour), cvstream);
	//outTex_.unmap(out_->stream());

	cudaSafeCall(cudaGraphicsUnmapResources(1, &cuda_res_, out_->stream()));
}

void GLRender::blend(ftl::codecs::Channel c) {

}

void GLRender::cancel() {

}
