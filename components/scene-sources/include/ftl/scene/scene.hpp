#ifndef _FTL_RECONSTRUCT_SCENE_HPP_
#define _FTL_RECONSTRUCT_SCENE_HPP_

namespace ftl {
namespace scene {

class Scene {
    public:
    Scene();
    virtual ~Scene();

    virtual bool render(ftl::rgbd::Source *, ftl::rgbd::Frame &)=0;

	virtual bool encode(std::vector<uint8_t> &)=0;
	virtual bool decode(const std::vector<uint8_t> &)=0;
};

}  // scene
}  // ftl

#endif  // _FTL_RECONSTRUCT_SCENE_HPP_
