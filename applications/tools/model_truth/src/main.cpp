#include <ftl/configuration.hpp>
#include <ftl/streams/filestream.hpp>
#include <ftl/streams/parsers.hpp>
#include <ftl/render/assimp_render.hpp>

#include <vector>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <GLFW/glfw3.h>

using std::vector;
using std::string;
using ftl::codecs::Channel;

struct CameraSpecs {
    ftl::rgbd::Camera intrinsics1;
    ftl::rgbd::Camera intrinsics2;
    Eigen::Matrix4d pose;
};

int main(int argc, char **argv) {
	auto *root = ftl::configure(argc, argv, "tools_default");

    vector<CameraSpecs> cameras;
	vector<ftl::rgbd::Frame> frames;

    // Use existing FTL as pose and intrinsics source
    ftl::stream::File *inftl = ftl::create<ftl::stream::File>(root, "input_ftl");
    inftl->set("filename", root->value("in", std::string("")));

    inftl->onPacket([&cameras](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
        if (spkt.channel == Channel::Pose) {
            if (cameras.size() <= spkt.frameNumber()) cameras.resize(spkt.frameNumber()+1);
            CameraSpecs &spec = cameras[spkt.frameNumber()];

            spec.pose = ftl::stream::parsePose(pkt);

            LOG(INFO) << "HAVE POSE: " << spec.pose;
        } else if (spkt.channel == Channel::Calibration) {
            if (cameras.size() <= spkt.frameNumber()) cameras.resize(spkt.frameNumber()+1);
            CameraSpecs &spec = cameras[spkt.frameNumber()];

            spec.intrinsics1 = ftl::stream::parseCalibration(pkt);

            LOG(INFO) << "HAVE INTRIN: " << spec.intrinsics1.width << "x" << spec.intrinsics1.height;
        }
    });
    inftl->set("looping", false);
    inftl->begin(false);
    inftl->tick(ftl::timer::get_time()+10000);  // Read 10 seconds of FTL file
    inftl->end();

    // Load a model using ASSIMP
    auto *scene = ftl::create<ftl::render::AssimpScene>(root, "scene");
	scene->set("model", root->value("model", std::string("")));
	auto *render = ftl::create<ftl::render::AssimpRenderer>(root, "render");
	render->setScene(scene);

    // Generate OpenGL context and frame buffer
    GLFWwindow *window;
    if (!glfwInit()) {
        LOG(ERROR) << "Could not init GL window";
        return -1;
    }

    glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
    window = glfwCreateWindow(640, 480, "FTL", NULL, NULL);

    if (!window) {
        glfwTerminate();
        LOG(ERROR) << "Could not create GL window";
        return -1;
    }

    glfwMakeContextCurrent(window);

	scene->load();
	frames.resize(cameras.size());

    // Render each view into framebuffer
	for (size_t i=0; i<cameras.size(); ++i) {
		render->setScene(scene);
		render->begin(frames[i], Channel::Colour);
		render->render();
		render->end();
	}

    // Download each view and encode as JPG / PNG colour depth pairs

    // Allow options to introduce distortion and noise into depth maps

    // Use a ground truth channel?

    glfwTerminate();
    return 0;
}
