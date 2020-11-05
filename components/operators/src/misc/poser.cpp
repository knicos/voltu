#include <ftl/operators/poser.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::operators::Poser;
using ftl::codecs::Channel;
using ftl::codecs::Shape3DType;
using std::string;

static SHARED_MUTEX smtx;
std::unordered_map<std::string,ftl::operators::Poser::PoseState> Poser::pose_db__;
std::unordered_map<int,std::list<ftl::codecs::Shape3D*>> Poser::fs_shapes__;

Poser::Poser(ftl::operators::Graph *g, ftl::Configurable *cfg) : ftl::operators::Operator(g, cfg) {

}

Poser::~Poser() {

}

void Poser::add(const ftl::codecs::Shape3D &t, ftl::data::FrameID id) {
	std::string idstr;
	switch(t.type) {
	case Shape3DType::ARUCO         : idstr = "aruco-"; break;
	case Shape3DType::CAMERA		: idstr = "camera-"; break;
	case Shape3DType::CURSOR		: idstr = "cursor-"; break;
	default                         : idstr = "unk-"; break;
	}

	idstr += std::to_string(id.frameset()) + string("-") + std::to_string(id.source()) + string("-") + std::to_string(t.id);

	//auto pose = t.pose.cast<double>();  // f.getPose() * 

	UNIQUE_LOCK(smtx, lk);
	auto p = pose_db__.find(idstr);
	if (p == pose_db__.end()) {
		ftl::operators::Poser::PoseState ps;
		ps.shape = t;
		ps.locked = false;
		pose_db__.emplace(std::make_pair(idstr,ps));
		LOG(INFO) << "POSE ID: " << idstr;
		fs_shapes__[id.frameset()].push_back(&pose_db__[idstr].shape);
	} else {
		// TODO: Merge poses
		if (!(*p).second.locked) (*p).second.shape = t;
		//LOG(INFO) << "POSE ID: " << idstr;
	}
}

std::list<ftl::codecs::Shape3D*> Poser::getAll(int32_t fsid) {
	SHARED_LOCK(smtx, lk);
	if (fs_shapes__.count(fsid)) {
		return fs_shapes__[fsid];
	}
	return {};
}

bool Poser::get(const std::string &name, Eigen::Matrix4d &pose) {
	SHARED_LOCK(smtx, lk);
	auto p = pose_db__.find(name);
	if (p != pose_db__.end()) {
		pose = (*p).second.shape.pose.cast<double>();
		return true;
	} else {
		LOG(WARNING) << "Pose not found: " << name;
		return false;
	}
}

bool Poser::apply(ftl::rgbd::FrameSet &in, ftl::rgbd::FrameSet &out, cudaStream_t stream) {
    if (in.hasChannel(Channel::Shapes3D)) {
        const auto &transforms = in.get<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);

		//LOG(INFO) << "Found shapes 3D global: " << (int)transforms.size();

        for (auto &t : transforms) {
        //    LOG(INFO) << "Have FS transform: " << t.label;
			add(t, in.id());
        }
    }

	for (size_t i=0; i<in.frames.size(); ++i) {
        //if (in.hasFrame(i)) {
            auto &f = in.frames[i].cast<ftl::rgbd::Frame>();

            if (f.hasChannel(Channel::Shapes3D)) {
                const auto &transforms = f.get<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);

				//LOG(INFO) << "Found shapes 3D: " << (int)transforms.size();

                for (auto &t : transforms) {
                    add(t, f.id());
                }
            }

			if (f.hasChannel(Channel::Pose)) {
				ftl::codecs::Shape3D cam;
				cam.id = 0;
				cam.label = f.name();
				cam.pose = f.getPose().cast<float>();
				cam.type = ftl::codecs::Shape3DType::CAMERA;
				add(cam, f.id());
			}
        //}
    }

	SHARED_LOCK(smtx, lk);
    string pose_ident = config()->value("pose_ident",string("default"));
    if (pose_ident != "default") {
        auto p = pose_db__.find(pose_ident);
        if (p != pose_db__.end()) {
			(*p).second.locked = config()->value("locked",false);

			Eigen::Matrix4d pose = (*p).second.shape.pose.cast<double>();

			if (in.frames.size() == 1) {
				auto response = in.frames[0].response();
				auto &rgbdf = response.cast<ftl::rgbd::Frame>();
				rgbdf.setPose() = (config()->value("inverse",false)) ? pose.inverse() : pose;
			} else {
            	in.cast<ftl::rgbd::Frame>().setPose() = (config()->value("inverse",false)) ? pose.inverse() : pose;
			}
        } else {
            LOG(WARNING) << "Pose not found: " << pose_ident;
        }
    }

	return true;
}

bool Poser::apply(ftl::rgbd::Frame &in, ftl::rgbd::Frame &out, cudaStream_t stream) {
	auto &f = in;

	if (f.hasChannel(Channel::Shapes3D)) {
		const auto &transforms = f.get<std::list<ftl::codecs::Shape3D>>(Channel::Shapes3D);

		for (auto &t : transforms) {
			add(t, f.id());
		}
	}

	if (f.hasChannel(Channel::Pose)) {
		ftl::codecs::Shape3D cam;
		cam.id = 0;
		cam.label = f.name();
		cam.pose = f.getPose().cast<float>();
		cam.type = ftl::codecs::Shape3DType::CAMERA;
		add(cam, f.id());
	}
	return true;
}