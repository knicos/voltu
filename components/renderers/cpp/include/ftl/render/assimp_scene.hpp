#ifndef _FTL_RENDER_ASSIMPSCENE_HPP_
#define _FTL_RENDER_ASSIMPSCENE_HPP_

#include <ftl/configurable.hpp>
#include <assimp/scene.h>
#include <nanogui/glutil.h>
#include <Eigen/Eigen>

#include <vector>
#include <map>
#include <unordered_map>

namespace ftl {
namespace render {

struct GLMesh {
	int material;
	std::vector<int> indices;
	Eigen::Matrix4d transform;
};

struct VBOData {
	float vertex[3];
	float normal[3];
	unsigned char colour[4];
	short uv[2];
};

class AssimpScene : public ftl::Configurable {
	public:
	explicit AssimpScene(nlohmann::json &config);
	~AssimpScene();

	void load();

	const aiScene *scene;
	std::map<std::string, GLuint*> textureIdMap;	// map image filenames to textureIds
	std::vector<GLuint> textureIds;
	std::vector<VBOData> vbo;  // Combined vertex data
	std::vector<GLMesh> meshes; // map materials to indicies vectors

	private:
	void _freeTextureIds();
	bool _loadGLTextures();
};

}
}

#endif
