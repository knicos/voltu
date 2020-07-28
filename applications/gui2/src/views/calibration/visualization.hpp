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

static void drawFloorPlan(NVGcontext* ctx, nanogui::Widget* parent,
		const std::vector<ftl::calibration::CalibrationData::Extrinsic>& calib,
		const std::vector<std::string>& names = {},
		int origin=0) {

	float minx = INFINITY;
	float miny = INFINITY;
	float maxx = -INFINITY;
	float maxy = -INFINITY;

	std::vector<cv::Point2f> points(calib.size());
	for (unsigned int i = 0; i < points.size(); i++) {
		// xz, assume floor on y-plane
		float x = calib[i].tvec[0];
		float y = calib[i].tvec[2];
		points[i] = {x, y};
		minx = std::min(minx, x);
		miny = std::min(miny, y);
		maxx = std::max(maxx, x);
		maxy = std::max(maxy, y);
	}

	float w = parent->width();
	float sx = w/(maxx - minx);
	float h = parent->height();
	float sy = h/(maxy - miny);
	float s = min(sx, sy);

	nanogui::Vector2f apos = parent->absolutePosition().cast<float>() + nanogui::Vector2f{w/2.0f, h/2.0f};

	for (unsigned int i = 0; i < points.size(); i++) {
		float x = points[i].x*s + apos.x();
		float y = points[i].y*s + apos.y();
		// TODO: draw triangles (rotate by camera rotation)
		nvgBeginPath(ctx);
		nvgCircle(ctx, x, y, 2.5);
		nvgStrokeColor(ctx, nvgRGBA(0, 0, 0, 255));
		nvgStrokeWidth(ctx, 1.0f);
		nvgStroke(ctx);
	}
}
