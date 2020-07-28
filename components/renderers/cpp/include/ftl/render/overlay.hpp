#ifndef _FTL_GUI_OVERLAY_HPP_
#define _FTL_GUI_OVERLAY_HPP_

#include <opencv2/core/mat.hpp>
#include <Eigen/Eigen>
#include <ftl/rgbd/frameset.hpp>
#include <nanogui/glutil.h>

struct NVGcontext;

namespace ftl {
namespace overlay {

enum class Shape {
    BOX,
    CAMERA,
    XZPLANE,
    GRID,
    AXIS
};

class Overlay : public ftl::Configurable {
	public:
	explicit Overlay(nlohmann::json &config);
	~Overlay();

	//void apply(ftl::rgbd::FrameSet &fs, cv::Mat &out, ftl::rgbd::FrameState &state);

	void draw(NVGcontext *, ftl::data::FrameSet &fs, ftl::rgbd::Frame &frame, const Eigen::Vector2f &);

	private:
	nanogui::GLShader oShader;
	bool init_;

    std::vector<float3> shape_verts_;
    std::vector<unsigned int> shape_tri_indices_;
    std::unordered_map<ftl::overlay::Shape, std::tuple<int,int,int,int>> shapes_;

    void _createShapes();
    void _drawFilledShape(ftl::overlay::Shape shape, const Eigen::Matrix4d &pose, float scale, uchar4 colour);
    void _drawOutlinedShape(Shape shape, const Eigen::Matrix4d &pose, const Eigen::Vector3f &scale, uchar4 fill, uchar4 outline);
    void _drawAxis(const Eigen::Matrix4d &pose, const Eigen::Vector3f &scale);
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

void drawFilledBox(
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
