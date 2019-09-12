#ifndef _FTL_RECONSTRUCT_SCENE_HPP_
#define _FTL_RECONSTRUCT_SCENE_HPP_

namespace ftl {

class Scene {
    public:
    Scene();
    ~Scene();

    void getFrame(eigen::Matrix4f &pose, ftl::rgbd::Frame &);
    ftl::rgbd::Frame getFrame(eigen::Matrix4f &pose);

    void getFrameSet(ftl::rgbd::FrameSet &);
    ftl::rgbd::FrameSet &getFrameSet();
};

}

#endif  // _FTL_RECONSTRUCT_SCENE_HPP_
