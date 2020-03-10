#ifndef _FTL_RENDER_VBO_HPP_
#define _FTL_RENDER_VBO_HPP_

namespace ftl {
namespace render {

/**
 * OpenGL Vertex Buffer Object wrapper.
 */
class VBO {
	public:
	VBO();
	~VBO();

	void begin();
	void end();

	void bind();
	void unbind();

	void writeVertex3D(float x, float y, float z);
	void writeVertex3D(const float3 &v);
	void writeVertex3D(const Eigen::Vertex3f &v);
	void writeVertices3D(float *v, size_t c);
	void writeVertices3D(float3 *v, size_t c);

	void writeColour(uchar r, uchar g, uchar b, uchar a);
	void writeColour(uchar4 c);
	void writeColour(float r, float g, float b, float a);
	void writeColours(float *v, size_t);
	void writeColours(uchar4 *c, size_t);

	void writeTexCoords(float u, float v);
	void writeTexCoords(const float2 &uv);
};

}
}

#endif