#pragma once

#include "../../widgets/imageview.hpp"

#include <ftl/calibration/structures.hpp>

/** Draw Chessboard Corners with OpenGL to ImageView widget. */
template<typename T>
static void drawChessboardCorners(NVGcontext* ctx, ftl::gui2::ImageView* imview, const std::vector<T>& points) {
	if (points.size() == 0) { return; }

	nanogui::Vector2f wpos = imview->absolutePosition().cast<float>();
	nanogui::Vector2f wsize = imview->sizeF();
	nanogui::Vector2f apos = imview->positionForCoordinate({points[0].x, points[0].y}) + wpos;

	nvgShapeAntiAlias(ctx, 1);
	nvgScissor(ctx, wpos.x(), wpos.y(), wsize.x(), wsize.y());
	nvgBeginPath(ctx);
	nvgMoveTo(ctx, apos.x(), apos.y());
	for (unsigned int i = 1; i < points.size(); i++) {
		apos = imview->positionForCoordinate({points[i].x, points[i].y}) + wpos;
		nvgLineTo(ctx, apos.x(), apos.y());
	}
	nvgStrokeColor(ctx, nvgRGBA(255, 32, 32, 192));
	nvgStrokeWidth(ctx, 1.0f);
	nvgStroke(ctx);

	for (unsigned int i = 0; i < points.size(); i++) {
		apos = imview->positionForCoordinate({points[i].x, points[i].y}) + wpos;
		nvgBeginPath(ctx);
		nvgCircle(ctx, apos.x(), apos.y(), 2.5);
		nvgStrokeColor(ctx, nvgRGBA(0, 0, 0, 255));
		nvgStrokeWidth(ctx, 1.5f);
		nvgStroke(ctx);
		nvgBeginPath(ctx);
		nvgCircle(ctx, apos.x(), apos.y(), 2.5);
		nvgStrokeColor(ctx, nvgRGBA(255, 255, 255, 255));
		nvgStrokeWidth(ctx, 1.0f);
		nvgStroke(ctx);

	}
	nvgResetScissor(ctx);
}

static void drawTriangle(NVGcontext* ctx, const ftl::calibration::CalibrationData::Extrinsic &calib,
		const nanogui::Vector2f &pos, const nanogui::Vector2f offset, float scale, float sz=1.0f) {
	const int idx_x = 0;
	const int idx_y = 2;

	cv::Mat T = calib.matrix();
	cv::Vec4f p1(cv::Mat(T * cv::Vec4d{sz/2.0f, 0.0f, 0.0f, 1.0f}));
	cv::Vec4f p2(cv::Mat(T * cv::Vec4d{-sz/2.0f, 0.0f, 0.0f, 1.0f}));
	cv::Vec4f p3(cv::Mat(T * cv::Vec4d{0.0f, 0.0f, -sz*sqrtf(3.0f)/2.0f, 1.0f}));

	p1[idx_x] -= offset.x();
	p2[idx_x] -= offset.x();
	p3[idx_x] -= offset.x();
	p1[idx_y] -= offset.y();
	p2[idx_y] -= offset.y();
	p3[idx_y] -= offset.y();
	p1 *= scale;
	p2 *= scale;
	p3 *= scale;

	nvgBeginPath(ctx);

	// NOTE: flip x
	nvgMoveTo(ctx, pos.x() + p1[idx_x], pos.y() + p1[idx_y]);
	nvgLineTo(ctx, pos.x() + p2[idx_x], pos.y() + p2[idx_y]);
	nvgLineTo(ctx, pos.x() + p3[idx_x], pos.y() + p3[idx_y]);
	nvgLineTo(ctx, pos.x() + p1[idx_x], pos.y() + p1[idx_y]);
	if (calib.tvec == cv::Vec3d{0.0, 0.0, 0.0}) {
		nvgStrokeColor(ctx, nvgRGBA(255, 64, 64, 255));
	}
	else {
		nvgStrokeColor(ctx, nvgRGBA(255, 255, 255, 255));
	}
	nvgStrokeWidth(ctx, 1.0f);
	nvgStroke(ctx);
}

static void drawFloorPlan(NVGcontext* ctx, nanogui::Widget* parent,
		const std::vector<ftl::calibration::CalibrationData::Calibration>& calib,
		const std::vector<std::string>& names = {},
		int origin=0) {

	float minx = INFINITY;
	float miny = INFINITY;
	float maxx = -INFINITY;
	float maxy = -INFINITY;
	cv::Vec3f center = {0.0f, 0.0f};
	std::vector<cv::Point2f> points(calib.size());
	for (unsigned int i = 0; i < points.size(); i++) {
		const auto& extrinsic = calib[i].extrinsic;
		// xz, assume floor on y-plane y = 0
		float x = extrinsic.tvec[0];
		float y = extrinsic.tvec[2];
		points[i] = {x, y};
		minx = std::min(minx, x);
		miny = std::min(miny, y);
		maxx = std::max(maxx, x);
		maxy = std::max(maxy, y);
		center += extrinsic.tvec;
	}
	center /= float(points.size());
	float w = parent->width();
	float dx = maxx - minx;
	float h = parent->height();
	float dy = maxy - miny;
	float s = min(w/dx, h/dy) * 0.8; // scale

	nanogui::Vector2f apos = parent->absolutePosition().cast<float>() + nanogui::Vector2f{w/2.0f, h/2.0f};
	nanogui::Vector2f off{center[0], center[2]};

	for (unsigned int i = 0; i < points.size(); i++) {
		drawTriangle(ctx, calib[i].extrinsic, apos, off, s, 0.3);
	}
}
