#include <ftl/render/overlay.hpp>

#include <opencv2/imgproc.hpp>

#include <ftl/codecs/shapes.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::overlay::Overlay;
using ftl::codecs::Channel;

Overlay::Overlay(nlohmann::json &config) : ftl::Configurable(config) {

}

Overlay::~Overlay() {

}

void Overlay::apply(ftl::rgbd::FrameSet &fs, cv::Mat &out, ftl::rgbd::FrameState &state) {
	over_depth_.create(out.size(), CV_32F);

	if (value("show_poses", false)) {
		for (size_t i=0; i<fs.frames.size(); ++i) {
			auto pose = fs.frames[i].getPose().inverse() * state.getPose();
			Eigen::Vector4d pos = pose.inverse() * Eigen::Vector4d(0,0,0,1);
			pos /= pos[3];

			auto name = fs.frames[i].get<std::string>("name");
			ftl::overlay::drawCamera(state.getLeft(), out, over_depth_, fs.frames[i].getLeftCamera(), pose, cv::Scalar(0,0,255,255), 0.2,value("show_frustrum", false));
			if (name) ftl::overlay::drawText(state.getLeft(), out, over_depth_, *name, pos, 0.5, cv::Scalar(0,0,255,255));
		}
	}

	if (value("show_shapes", false)) {
		if (fs.hasChannel(Channel::Shapes3D)) {
			std::vector<ftl::codecs::Shape3D> shapes;
			fs.get(Channel::Shapes3D, shapes);

			for (auto &s : shapes) {
				auto pose = s.pose.cast<double>().inverse() * state.getPose();
				Eigen::Vector4d pos = pose.inverse() * Eigen::Vector4d(0,0,0,1);
				pos /= pos[3];

				ftl::overlay::drawFilledBox(state.getLeft(), out, over_depth_, pose, cv::Scalar(0,0,255,100), s.size.cast<double>());
                ftl::overlay::drawBox(state.getLeft(), out, over_depth_, pose, cv::Scalar(0,0,255,255), s.size.cast<double>());
				ftl::overlay::drawText(state.getLeft(), out, over_depth_, s.label, pos, 0.5, cv::Scalar(0,0,255,100));
			}
		}

		for (size_t i=0; i<fs.frames.size(); ++i) {
			if (fs.frames[i].hasChannel(Channel::Shapes3D)) {
				std::vector<ftl::codecs::Shape3D> shapes;
				fs.frames[i].get(Channel::Shapes3D, shapes);

				for (auto &s : shapes) {
					auto pose = s.pose.cast<double>().inverse() * state.getPose();
					Eigen::Vector4d pos = pose.inverse() * Eigen::Vector4d(0,0,0,1);
					pos /= pos[3];

					ftl::overlay::drawBox(state.getLeft(), out, over_depth_, pose, cv::Scalar(0,0,255,100), s.size.cast<double>());
					ftl::overlay::drawText(state.getLeft(), out, over_depth_, s.label, pos, 0.5, cv::Scalar(0,0,255,100));
				}
			}
		}
	}

	//cv::flip(out, out, 0);
}

void ftl::overlay::draw3DLine(
        const ftl::rgbd::Camera &cam,
        cv::Mat &colour,
        cv::Mat &depth,
        const Eigen::Vector4d &begin,
        const Eigen::Vector4d &end,
        const cv::Scalar &linecolour) {


    auto begin_pos = cam.camToScreen<int2>(make_float3(begin[0], begin[1], begin[2]));
    auto end_pos = cam.camToScreen<int2>(make_float3(end[0], end[1], end[2]));

    cv::LineIterator lineit(colour, cv::Point(begin_pos.x, begin_pos.y), cv::Point(end_pos.x, end_pos.y));
    double z_grad = (end[2] - begin[2]) / lineit.count;
    double current_z = begin[2];

    for(int i = 0; i < lineit.count; i++, ++lineit) {
        colour.at<cv::Vec4b>(lineit.pos()) = linecolour;
        depth.at<float>(lineit.pos()) = current_z;
        current_z += z_grad;
    }
}

void ftl::overlay::drawPoseBox(
        const ftl::rgbd::Camera &cam,
        cv::Mat &colour,
        cv::Mat &depth,
        const Eigen::Matrix4d &pose,
        const cv::Scalar &linecolour,
        double size) {

    double size2 = size/2.0;

    Eigen::Vector4d p001 = pose.inverse() * Eigen::Vector4d(size2,size2,-size2,1);
    Eigen::Vector4d p011 = pose.inverse() * Eigen::Vector4d(size2,-size2,-size2,1);
    Eigen::Vector4d p111 = pose.inverse() * Eigen::Vector4d(-size2,-size2,-size2,1);
    Eigen::Vector4d p101 = pose.inverse() * Eigen::Vector4d(-size2,size2,-size2,1);
    Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(-size2,-size2,size2,1);
    Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(-size2,size2,size2,1);
    Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(size2,-size2,size2,1);
    Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(size2,size2,size2,1);

    p001 /= p001[3];
    p011 /= p011[3];
    p111 /= p111[3];
    p101 /= p101[3];
    p110 /= p110[3];
    p100 /= p100[3];
    p010 /= p010[3];
    p000 /= p000[3];

    if (p001[2] < 0.1 || p011[2] < 0.1 || p111[2] < 0.1 || p101[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

    draw3DLine(cam, colour, depth, p000, p001, linecolour);
    draw3DLine(cam, colour, depth, p000, p010, linecolour);
    draw3DLine(cam, colour, depth, p000, p100, linecolour);

    draw3DLine(cam, colour, depth, p001, p011, linecolour);
    draw3DLine(cam, colour, depth, p001, p101, linecolour);

    draw3DLine(cam, colour, depth, p010, p011, linecolour);
    draw3DLine(cam, colour, depth, p010, p110, linecolour);

    draw3DLine(cam, colour, depth, p100, p101, linecolour);
    draw3DLine(cam, colour, depth, p100, p110, linecolour);

    draw3DLine(cam, colour, depth, p101, p111, linecolour);
    draw3DLine(cam, colour, depth, p110, p111, linecolour);
    draw3DLine(cam, colour, depth, p011, p111, linecolour);
}

void ftl::overlay::drawBox(
        const ftl::rgbd::Camera &cam,
        cv::Mat &colour,
        cv::Mat &depth,
        const Eigen::Matrix4d &pose,
        const cv::Scalar &linecolour,
        const Eigen::Vector3d &size) {

    double size2x = size[0]/2.0;
	double size2y = size[1]/2.0;
	double size2z = size[2]/2.0;

    Eigen::Vector4d p001 = pose.inverse() * Eigen::Vector4d(size2x,size2y,-size2z,1);
    Eigen::Vector4d p011 = pose.inverse() * Eigen::Vector4d(size2x,-size2y,-size2z,1);
    Eigen::Vector4d p111 = pose.inverse() * Eigen::Vector4d(-size2x,-size2y,-size2z,1);
    Eigen::Vector4d p101 = pose.inverse() * Eigen::Vector4d(-size2x,size2y,-size2z,1);
    Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(-size2x,-size2y,size2z,1);
    Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(-size2x,size2y,size2z,1);
    Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(size2x,-size2y,size2z,1);
    Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(size2x,size2y,size2z,1);

    p001 /= p001[3];
    p011 /= p011[3];
    p111 /= p111[3];
    p101 /= p101[3];
    p110 /= p110[3];
    p100 /= p100[3];
    p010 /= p010[3];
    p000 /= p000[3];

    if (p001[2] < 0.1 || p011[2] < 0.1 || p111[2] < 0.1 || p101[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

    draw3DLine(cam, colour, depth, p000, p001, linecolour);
    draw3DLine(cam, colour, depth, p000, p010, linecolour);
    draw3DLine(cam, colour, depth, p000, p100, linecolour);

    draw3DLine(cam, colour, depth, p001, p011, linecolour);
    draw3DLine(cam, colour, depth, p001, p101, linecolour);

    draw3DLine(cam, colour, depth, p010, p011, linecolour);
    draw3DLine(cam, colour, depth, p010, p110, linecolour);

    draw3DLine(cam, colour, depth, p100, p101, linecolour);
    draw3DLine(cam, colour, depth, p100, p110, linecolour);

    draw3DLine(cam, colour, depth, p101, p111, linecolour);
    draw3DLine(cam, colour, depth, p110, p111, linecolour);
    draw3DLine(cam, colour, depth, p011, p111, linecolour);
}

void ftl::overlay::drawFilledBox(
        const ftl::rgbd::Camera &cam,
        cv::Mat &colour,
        cv::Mat &depth,
        const Eigen::Matrix4d &pose,
        const cv::Scalar &linecolour,
        const Eigen::Vector3d &size) {

    double size2x = size[0]/2.0;
	double size2y = size[1]/2.0;
	double size2z = size[2]/2.0;

    Eigen::Vector4d p001 = pose.inverse() * Eigen::Vector4d(size2x,size2y,-size2z,1);
    Eigen::Vector4d p011 = pose.inverse() * Eigen::Vector4d(size2x,-size2y,-size2z,1);
    Eigen::Vector4d p111 = pose.inverse() * Eigen::Vector4d(-size2x,-size2y,-size2z,1);
    Eigen::Vector4d p101 = pose.inverse() * Eigen::Vector4d(-size2x,size2y,-size2z,1);
    Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(-size2x,-size2y,size2z,1);
    Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(-size2x,size2y,size2z,1);
    Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(size2x,-size2y,size2z,1);
    Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(size2x,size2y,size2z,1);

    p001 /= p001[3];
    p011 /= p011[3];
    p111 /= p111[3];
    p101 /= p101[3];
    p110 /= p110[3];
    p100 /= p100[3];
    p010 /= p010[3];
    p000 /= p000[3];

    if (p001[2] < 0.1 || p011[2] < 0.1 || p111[2] < 0.1 || p101[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

    std::array<cv::Point, 4> pts;

    auto p = cam.camToScreen<int2>(make_float3(p000[0], p000[1], p000[2]));
    pts[0] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p001[0], p001[1], p001[2]));
    pts[1] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p011[0], p011[1], p011[2]));
    pts[2] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p010[0], p010[1], p010[2]));
    pts[3] = cv::Point(p.x, p.y);
    cv::fillConvexPoly(colour, pts, linecolour);

    p = cam.camToScreen<int2>(make_float3(p100[0], p100[1], p100[2]));
    pts[0] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p101[0], p101[1], p101[2]));
    pts[1] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p111[0], p111[1], p111[2]));
    pts[2] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p110[0], p110[1], p110[2]));
    pts[3] = cv::Point(p.x, p.y);
    cv::fillConvexPoly(colour, pts, linecolour);

    p = cam.camToScreen<int2>(make_float3(p000[0], p000[1], p000[2]));
    pts[0] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p010[0], p010[1], p010[2]));
    pts[1] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p110[0], p110[1], p110[2]));
    pts[2] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p100[0], p100[1], p100[2]));
    pts[3] = cv::Point(p.x, p.y);
    cv::fillConvexPoly(colour, pts, linecolour);

    p = cam.camToScreen<int2>(make_float3(p001[0], p001[1], p001[2]));
    pts[0] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p011[0], p011[1], p011[2]));
    pts[1] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p111[0], p111[1], p111[2]));
    pts[2] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p101[0], p101[1], p101[2]));
    pts[3] = cv::Point(p.x, p.y);
    cv::fillConvexPoly(colour, pts, linecolour);

    p = cam.camToScreen<int2>(make_float3(p000[0], p000[1], p000[2]));
    pts[0] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p001[0], p001[1], p001[2]));
    pts[1] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p101[0], p101[1], p101[2]));
    pts[2] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p100[0], p100[1], p100[2]));
    pts[3] = cv::Point(p.x, p.y);
    cv::fillConvexPoly(colour, pts, linecolour);

    p = cam.camToScreen<int2>(make_float3(p010[0], p010[1], p010[2]));
    pts[0] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p011[0], p011[1], p011[2]));
    pts[1] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p111[0], p111[1], p111[2]));
    pts[2] = cv::Point(p.x, p.y);
    p = cam.camToScreen<int2>(make_float3(p110[0], p110[1], p110[2]));
    pts[3] = cv::Point(p.x, p.y);
    cv::fillConvexPoly(colour, pts, linecolour);
}

void ftl::overlay::drawRectangle(
        const ftl::rgbd::Camera &cam,
        cv::Mat &colour,
        cv::Mat &depth,
        const Eigen::Matrix4d &pose,
        const cv::Scalar &linecolour,
        double width, double height) {

    double width2 = width/2.0;
    double height2 = height/2.0;

    Eigen::Vector4d p001 = pose.inverse() * Eigen::Vector4d(width2,height2,0,1);
    Eigen::Vector4d p011 = pose.inverse() * Eigen::Vector4d(width2,-height2,0,1);
    Eigen::Vector4d p111 = pose.inverse() * Eigen::Vector4d(-width2,-height2,0,1);
    Eigen::Vector4d p101 = pose.inverse() * Eigen::Vector4d(-width2,height2,0,1);

    p001 /= p001[3];
    p011 /= p011[3];
    p111 /= p111[3];
    p101 /= p101[3];

    if (p001[2] < 0.1 || p011[2] < 0.1 || p111[2] < 0.1 || p101[2] < 0.1) return;

    draw3DLine(cam, colour, depth, p001, p011, linecolour);
    draw3DLine(cam, colour, depth, p001, p101, linecolour);
    draw3DLine(cam, colour, depth, p101, p111, linecolour);
    draw3DLine(cam, colour, depth, p011, p111, linecolour);
}

void ftl::overlay::drawPoseCone(
        const ftl::rgbd::Camera &cam,
        cv::Mat &colour,
        cv::Mat &depth,
        const Eigen::Matrix4d &pose,
        const cv::Scalar &linecolour,
        double size) {

    double size2 = size;

    Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(-size2,-size2,size2,1);
    Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(-size2,size2,size2,1);
    Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(size2,-size2,size2,1);
    Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(size2,size2,size2,1);
    Eigen::Vector4d origin = pose.inverse() * Eigen::Vector4d(0,0,0,1);

    p110 /= p110[3];
    p100 /= p100[3];
    p010 /= p010[3];
    p000 /= p000[3];
    origin /= origin[3];

    if (origin[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

    draw3DLine(cam, colour, depth, p000, origin, linecolour);
    draw3DLine(cam, colour, depth, p000, p010, linecolour);
    draw3DLine(cam, colour, depth, p000, p100, linecolour);

    draw3DLine(cam, colour, depth, p010, origin, linecolour);
    draw3DLine(cam, colour, depth, p010, p110, linecolour);

    draw3DLine(cam, colour, depth, p100, origin, linecolour);
    draw3DLine(cam, colour, depth, p100, p110, linecolour);

    draw3DLine(cam, colour, depth, p110, origin, linecolour);
}

void ftl::overlay::drawCamera(
        const ftl::rgbd::Camera &vcam,
        cv::Mat &colour,
        cv::Mat &depth,
        const ftl::rgbd::Camera &camera,
        const Eigen::Matrix4d &pose,
        const cv::Scalar &linecolour,
        double scale, bool frustrum) {

    //double size2 = size;

    const auto &params = camera;
    double width = (static_cast<double>(params.width) / static_cast<double>(params.fx)) * scale;
    double height = (static_cast<double>(params.height) / static_cast<double>(params.fx)) * scale;
    double width2 = width / 2.0;
    double height2 = height / 2.0;

    double principx = (((static_cast<double>(params.width) / 2.0) + params.cx) / static_cast<double>(params.fx)) * scale;
    double principy = (((static_cast<double>(params.height) / 2.0) + params.cy) / static_cast<double>(params.fx)) * scale;

    auto ptcoord = params.screenToCam(0,0,scale);
    Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
    ptcoord = params.screenToCam(0,params.height,scale);
    Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
    ptcoord = params.screenToCam(params.width,0,scale);
    Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
    ptcoord = params.screenToCam(params.width,params.height,scale);
    Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
    Eigen::Vector4d origin = pose.inverse() * Eigen::Vector4d(0,0,0,1);

    p110 /= p110[3];
    p100 /= p100[3];
    p010 /= p010[3];
    p000 /= p000[3];
    origin /= origin[3];

    if (origin[2] < 0.1 || p110[2] < 0.1 || p100[2] < 0.1 || p010[2] < 0.1 || p000[2] < 0.1) return;

    draw3DLine(vcam, colour, depth, p000, origin, linecolour);
    draw3DLine(vcam, colour, depth, p000, p010, linecolour);
    draw3DLine(vcam, colour, depth, p000, p100, linecolour);

    draw3DLine(vcam, colour, depth, p010, origin, linecolour);
    draw3DLine(vcam, colour, depth, p010, p110, linecolour);

    draw3DLine(vcam, colour, depth, p100, origin, linecolour);
    draw3DLine(vcam, colour, depth, p100, p110, linecolour);

    draw3DLine(vcam, colour, depth, p110, origin, linecolour);

    if (frustrum) {
        const double fscale = 16.0;
        ptcoord = params.screenToCam(0,0,fscale);
        Eigen::Vector4d f110 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
        ptcoord = params.screenToCam(0,params.height,fscale);
        Eigen::Vector4d f100 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
        ptcoord = params.screenToCam(params.width,0,fscale);
        Eigen::Vector4d f010 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);
        ptcoord = params.screenToCam(params.width,params.height,fscale);
        Eigen::Vector4d f000 = pose.inverse() * Eigen::Vector4d(ptcoord.x,ptcoord.y,ptcoord.z,1);

        f110 /= f110[3];
        f100 /= f100[3];
        f010 /= f010[3];
        f000 /= f000[3];

        if (f110[2] < 0.1 || f100[2] < 0.1 || f010[2] < 0.1 || f000[2] < 0.1) return;

        draw3DLine(vcam, colour, depth, f000, p000, cv::Scalar(0,255,0,0));
        draw3DLine(vcam, colour, depth, f010, p010, cv::Scalar(0,255,0,0));
        draw3DLine(vcam, colour, depth, f100, p100, cv::Scalar(0,255,0,0));
        draw3DLine(vcam, colour, depth, f110, p110, cv::Scalar(0,255,0,0));

        draw3DLine(vcam, colour, depth, f000, f010, cv::Scalar(0,255,0,0));
        draw3DLine(vcam, colour, depth, f000, f100, cv::Scalar(0,255,0,0));
        draw3DLine(vcam, colour, depth, f010, f110, cv::Scalar(0,255,0,0));
        draw3DLine(vcam, colour, depth, f100, f110, cv::Scalar(0,255,0,0));
    }
}

void ftl::overlay::drawText(
        const ftl::rgbd::Camera &cam,
        cv::Mat &colour,
        cv::Mat &depth,
        const std::string &text,
        const Eigen::Vector4d &pos,
        double size,
        const cv::Scalar &textcolour) {

    auto pt = cam.camToScreen<int2>(make_float3(pos[0], pos[1], pos[2]));
    if (pos[2] < 0.1) return;
    cv::putText(colour, text, cv::Point(pt.x, colour.rows-pt.y), 0, size, textcolour, 1, cv::LINE_8, true);
}
