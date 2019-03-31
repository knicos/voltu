#ifndef _FTL_ALGORITHMS_NP_ATTENTION_HPP_
#define _FTL_ALGORITHMS_NP_ATTENTION_HPP_

namespace ftl {
namespace gpu {

struct AttentionItem {
	float factor;
	HisteresisTexture<float> source;
}

class Attention {
	public:
	Attention();
	
	void source(float factor, const HisteresisTexture<float> &ht);
	
	void update();
	
	cudaTextureObject_t cudaTexture();
	TextureObject<float> texture();
	
	private:
	std::vector<Component> components_;
};

}
}

#endif // _FTL_ALGORITHMS_NP_ATTENTION_HPP_

