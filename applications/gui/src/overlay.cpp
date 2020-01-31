#include "overlay.hpp"

#include <opencv2/imgproc.hpp>

void ftl::overlay::draw3DLine(
        const ftl::rgbd::Camera &cam,
        cv::Mat &colour,
        cv::Mat &depth,
        const Eigen::Vector4d &begin,
        const Eigen::Vector4d &end,
        const cv::Scalar &linecolour) {


    auto begin_pos = cam.camToScreen<int2>(make_float3(begin[0], begin[1], begin[2]));
    auto end_pos = cam.camToScreen<int2>(make_float3(end[0], end[1], end[2]));

    cv::LineIterator lineit(colour, cv::Point(begin_pos.x, colour.rows-begin_pos.y), cv::Point(end_pos.x, colour.rows-end_pos.y));
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

    Eigen::Vector4d p110 = pose.inverse() * Eigen::Vector4d(-width2,-height2,scale,1);
    Eigen::Vector4d p100 = pose.inverse() * Eigen::Vector4d(-width2,height2,scale,1);
    Eigen::Vector4d p010 = pose.inverse() * Eigen::Vector4d(width2,-height2,scale,1);
    Eigen::Vector4d p000 = pose.inverse() * Eigen::Vector4d(width2,height2,scale,1);
    Eigen::Vector4d origin = pose.inverse() * Eigen::Vector4d(principx,principy,0,1);

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
        Eigen::Vector4d f110 = pose.inverse() * Eigen::Vector4d(-width2*fscale,-height2*fscale,scale*fscale,1);
        Eigen::Vector4d f100 = pose.inverse() * Eigen::Vector4d(-width2*fscale,height2*fscale,scale*fscale,1);
        Eigen::Vector4d f010 = pose.inverse() * Eigen::Vector4d(width2*fscale,-height2*fscale,scale*fscale,1);
        Eigen::Vector4d f000 = pose.inverse() * Eigen::Vector4d(width2*fscale,height2*fscale,scale*fscale,1);
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
