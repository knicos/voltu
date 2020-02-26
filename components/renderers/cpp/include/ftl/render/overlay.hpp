#ifndef _FTL_GUI_OVERLAY_HPP_
#define _FTL_GUI_OVERLAY_HPP_

#include <opencv2/core/mat.hpp>
#include <Eigen/Eigen>
#include <ftl/rgbd/frameset.hpp>

namespace ftl {
namespace overlay {

class Overlay : public ftl::Configurable {
	public:
	explicit Overlay(nlohmann::json &config);
	~Overlay();

	void apply(ftl::rgbd::FrameSet &fs, cv::Mat &out, ftl::rgbd::FrameState &state);

	private:
	cv::Mat over_depth_;
};

void draw3DLine(
    const ftl::rgbd::Camera &cam,
    cv::Mat &colour,
    cv::Mat &depth,
    const Eigen::Vector4d &begin,
    const Eigen::Vector4d &end,
    const cv::Scalar &linecolour);

void drawText(
    const ftl::rgbd::Camera &cam,
    cv::Mat &colour,
    cv::Mat &depth,
    const std::string &text,
    const Eigen::Vector4d &pos,
    double size,
    const cv::Scalar &textcolour);

/**
 * Draw a box at a given pose location and rotation.
 */
void drawPoseBox(
    const ftl::rgbd::Camera &cam,
    cv::Mat &colour,
    cv::Mat &depth,
    const Eigen::Matrix4d &pose,
    const cv::Scalar &linecolour,
    double size);

void drawBox(
    const ftl::rgbd::Camera &cam,
    cv::Mat &colour,
    cv::Mat &depth,
    const Eigen::Matrix4d &pose,
    const cv::Scalar &linecolour,
    const Eigen::Vector3d &size);

void drawRectangle(
        const ftl::rgbd::Camera &cam,
        cv::Mat &colour,
        cv::Mat &depth,
        const Eigen::Matrix4d &pose,
        const cv::Scalar &linecolour,
        double width, double height);

void drawPoseCone(
    const ftl::rgbd::Camera &cam,
    cv::Mat &colour,
    cv::Mat &depth,
    const Eigen::Matrix4d &pose,
    const cv::Scalar &linecolour,
    double size);

void drawCamera(
    const ftl::rgbd::Camera &cam,
    cv::Mat &colour,
    cv::Mat &depth,
    const ftl::rgbd::Camera &camera,
    const Eigen::Matrix4d &pose,
    const cv::Scalar &linecolour,
    double scale=1.0,
    bool frustrum=false);

}
}

#endif  // _FTL_GUI_OVERLAY_HPP_
