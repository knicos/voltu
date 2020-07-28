#include <ftl/operators/poser.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::operators::Poser;
using ftl::codecs::Channel;
using ftl::codecs::Shape3DType;
using std::string;

std::unordered_map<std::string,ftl::operators::Poser::PoseState> Poser::pose_db__;

Poser::Poser(ftl::Configurable *cfg) : ftl::operators::Operator(cfg) {

}

Poser::~Poser() {

}

void Poser::add(const ftl::codecs::Shape3D &t, int frameset, int frame) {
	std::string idstr;
	switch(t.type) {
	case Shape3DType::ARUCO         : idstr = "aruco-"; break;
	case Shape3DType::CAMERA		: idstr = "camera-"; break;
	default                         : idstr = "unk-"; break;
	}

	idstr += std::to_string(frameset) + string("-") + std::to_string(frame) + string("-") + std::to_string(t.id);

	auto pose = t.pose.cast<double>();  // f.getPose() * 

	auto p = pose_db__.find(idstr);
	if (p == pose_db__.end()) {
		ftl::operators::Poser::PoseState ps;
		ps.pose = pose;
		ps.locked = false;
		pose_db__.emplace(std::make_pair(idstr,ps));
		LOG(INFO) << "POSE ID: " << idstr;
	} else {
		// TODO: Merge poses
		if (!(*p).second.locked) (*p).second.pose = pose;
		//LOG(INFO) << "POSE ID: " << idstr;
	}
}

bool Poser::get(const std::string &name, Eigen::Matrix4d &pose) {
	auto p = pose_db__.find(name);
	if (p != pose_db__.end()) {
		pose = (*p).second.pose;
		return true;
	} else {
		LOG(WARNING) << "Pose not found: " << name;
		return false;
	}
}

bool Poser::set(const std::string &name, const Eigen::Matrix4d &pose) {
	auto p = pose_db__.find(name);
	if (p == pose_db__.end()) {
		ftl::operators::Poser::PoseState ps;
		ps.pose = pose;
		ps.locked = false;
		pose_db__.emplace(std::make_pair(name,ps));
		LOG(INFO) << "POSE ID: " << name;
	} else {
		// TODO: Merge poses
		if (!(*p).second.locked) (*p).second.pose = pose;
		//LOG(INFO) << "POSE ID: " << idstr;
	}
	return true;
}

bool Poser::apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) {
    if (in.hasChannel(Channel::Shapes3D)) {
        const auto &transforms = in.get<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);

		//LOG(INFO) << "Found shapes 3D global: " << (int)transforms.size();

        for (auto &t : transforms) {
        //    LOG(INFO) << "Have FS transform: " << t.label;
			add(t, in.frameset(), 255);
        }
    }

	for (size_t i=0; i<in.frames.size(); ++i) {
        //if (in.hasFrame(i)) {
            auto &f = in.frames[i].cast<ftl::rgbd::Frame>();

            if (f.hasChannel(Channel::Shapes3D)) {
                const auto &transforms = f.get<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);

				//LOG(INFO) << "Found shapes 3D: " << (int)transforms.size();

                for (auto &t : transforms) {
                    add(t, in.frameset(), i);
                }
            }

			if (f.hasChannel(Channel::Pose)) {
				ftl::codecs::Shape3D cam;
				cam.id = 0;
				cam.label = f.name();
				cam.pose = f.getPose().cast<float>();
				cam.type = ftl::codecs::Shape3DType::CAMERA;
				add(cam, in.frameset(), i);
			}
        //}
    }

    string pose_ident = config()->value("pose_ident",string("default"));
    if (pose_ident != "default") {
        auto p = pose_db__.find(pose_ident);
        if (p != pose_db__.end()) {
			(*p).second.locked = config()->value("locked",false);
            in.cast<ftl::rgbd::Frame>().setPose() = (config()->value("inverse",false)) ? (*p).second.pose.inverse() : (*p).second.pose;
        } else {
            LOG(WARNING) << "Pose not found: " << pose_ident;
        }
    }

	return true;
}