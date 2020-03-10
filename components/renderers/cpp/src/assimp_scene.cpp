#include <ftl/render/assimp_scene.hpp>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using ftl::render::AssimpScene;
using std::string;

static std::string getBasePath(const std::string& path) {
	size_t pos = path.find_last_of("\\/");
	return (std::string::npos == pos) ? "" : path.substr(0, pos + 1);
}

AssimpScene::AssimpScene(nlohmann::json &config) : ftl::Configurable(config) {

}

AssimpScene::~AssimpScene() {
	_freeTextureIds();
}

void AssimpScene::load() {
	if (scene) return;

	Assimp::Importer importer;
    scene = importer.ReadFile(value("model", std::string("")), aiProcessPreset_TargetRealtime_Quality);

    if (!scene) {
        throw FTL_Error("Could not load model: " << value("model", string("")));
    }

	_loadGLTextures();

	for (size_t n=0; n<scene->mNumMeshes; ++n) {
		const aiMesh* mesh = scene->mMeshes[n];

		vbo.reserve(vbo.size()+mesh->mNumVertices);
		for (size_t i=0; i<mesh->mNumVertices; ++i) {
			//vertices.push_back(mesh->mVertices)
		}
	}
}

void AssimpScene::_freeTextureIds() {
	textureIdMap.clear();
}

bool AssimpScene::_loadGLTextures() {
	_freeTextureIds();

    if (scene->HasTextures()) return true;

	/* getTexture Filenames and Numb of Textures */
	for (unsigned int m=0; m<scene->mNumMaterials; m++)
	{
		int texIndex = 0;
		aiReturn texFound = AI_SUCCESS;

		aiString path;	// filename

		while (texFound == AI_SUCCESS)
		{
			texFound = scene->mMaterials[m]->GetTexture(aiTextureType_DIFFUSE, texIndex, &path);
			textureIdMap[path.data] = NULL; //fill map with textures, pointers still NULL yet
			texIndex++;
		}
	}

	const size_t numTextures = textureIdMap.size();

	/* create and fill array with GL texture ids */
	//textureIds = new GLuint[numTextures];
	textureIds.resize(numTextures);
	glGenTextures(static_cast<GLsizei>(numTextures), textureIds.data()); /* Texture name generation */

	/* get iterator */
	std::map<std::string, GLuint*>::iterator itr = textureIdMap.begin();

	std::string basepath = getBasePath(value("model", std::string("")));
	for (size_t i=0; i<numTextures; i++) {

		//save IL image ID
		std::string filename = (*itr).first;  // get filename
		(*itr).second =  &textureIds[i];	  // save texture id for filename in map
		++itr;								  // next texture


		//ilBindImage(imageIds[i]); /* Binding of DevIL image name */
		std::string fileloc = basepath + filename;	/* Loading of image */
		//success = ilLoadImage(fileloc.c_str());
        int x, y, n;
		cv::Mat img = cv::imread(fileloc);
        unsigned char *data = img.data;

		if (!img.empty()) {
            // Convert every colour component into unsigned byte.If your image contains
            // alpha channel you can replace IL_RGB with IL_RGBA
            //success = ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE);
			/*if (!success)
			{
				abortGLInit("Couldn't convert image");
				return -1;
			}*/
            // Binding of texture name
            glBindTexture(GL_TEXTURE_2D, textureIds[i]);
			// redefine standard texture values
            // We will use linear interpolation for magnification filter
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
            // We will use linear interpolation for minifying filter
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
            // Texture specification
            glTexImage2D(GL_TEXTURE_2D, 0, n, x, y, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);// Texture specification.

            // we also want to be able to deal with odd texture dimensions
            glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
            glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
            glPixelStorei( GL_UNPACK_SKIP_PIXELS, 0 );
            glPixelStorei( GL_UNPACK_SKIP_ROWS, 0 );
        }
		else
		{
			/* Error occurred */
			//MessageBox(NULL, UTFConverter("Couldn't load Image: " + fileloc).c_wstr(), TEXT("ERROR"), MB_OK | MB_ICONEXCLAMATION);
			throw FTL_Error("Could no load texture image: " << fileloc);
		}
	}

	return true;
}