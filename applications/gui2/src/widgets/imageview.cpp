/*
	nanogui/imageview.cpp -- Widget used to display images.

	The image view widget was contributed by Stefan Ivanov.

	NanoGUI was developed by Wenzel Jakob <wenzel.jakob@epfl.ch>.
	The widget drawing code is based on the NanoVG demo application
	by Mikko Mononen.

	All rights reserved. Use of this source code is governed by a
	BSD-style license that can be found in the LICENSE.txt file.
*/

#include <nanogui/window.h>
#include <nanogui/screen.h>
#include <nanogui/theme.h>
#include <cmath>

#include <ftl/utility/gltexture.hpp>
#include "imageview.hpp"

using namespace nanogui;
using ftl::gui2::ImageView;
using ftl::gui2::FTLImageView;
using ftl::utility::GLTexture;

namespace {
	std::vector<std::string> tokenize(const std::string &string,
									  const std::string &delim = "\n",
									  bool includeEmpty = false) {
		std::string::size_type lastPos = 0, pos = string.find_first_of(delim, lastPos);
		std::vector<std::string> tokens;

		while (lastPos != std::string::npos) {
			std::string substr = string.substr(lastPos, pos - lastPos);
			if (!substr.empty() || includeEmpty)
				tokens.push_back(std::move(substr));
			lastPos = pos;
			if (lastPos != std::string::npos) {
				lastPos += 1;
				pos = string.find_first_of(delim, lastPos);
			}
		}

		return tokens;
	}

	constexpr char const *const defaultImageViewVertexShader =
		R"(#version 330
		uniform vec2 scaleFactor;
		uniform vec2 position;
		in vec2 vertex;
		out vec2 uv;
		void main() {
			uv = vertex;
			vec2 scaledVertex = (vertex * scaleFactor) + position;
			gl_Position  = vec4(2.0*scaledVertex.x - 1.0,
								1.0 - 2.0*scaledVertex.y,
								0.0, 1.0);

		})";

	constexpr char const *const defaultImageViewFragmentShader =
		R"(#version 330
		uniform sampler2D image;
		out vec4 color;
		in vec2 uv;
		void main() {
			color = texture(image, uv);
			color.w = 1;
		})";

}

ftl::gui2::ImageView::ImageView(Widget* parent, GLuint imageID)
	: Widget(parent), mImageID(imageID), mScale(1.0f), mOffset(Vector2f::Zero()),
	mFixedScale(false), mFixedOffset(false), mPixelInfoCallback(nullptr) {

	mImageSize = {0, 0};

	if (imageID != unsigned(-1)) {
		updateImageParameters();
	}

	mShader.init("ImageViewShader", defaultImageViewVertexShader,
				 defaultImageViewFragmentShader);

	MatrixXu indices(3, 2);
	indices.col(0) << 0, 1, 2;
	indices.col(1) << 2, 3, 1;

	MatrixXf vertices(2, 4);
	vertices.col(0) << 0, 0;
	vertices.col(1) << 1, 0;
	vertices.col(2) << 0, 1;
	vertices.col(3) << 1, 1;

	mShader.bind();
	mShader.uploadIndices(indices);
	mShader.uploadAttrib("vertex", vertices);
}

ftl::gui2::ImageView::~ImageView() {
	mShader.free();
}

void ftl::gui2::ImageView::bindImage(GLuint imageId) {
	if (imageId == unsigned(-1)) {
		return;
	}

	mImageID = imageId;
	updateImageParameters();

}

Vector2f ftl::gui2::ImageView::imageCoordinateAt(const Vector2f& position) const {
	auto imagePosition = position - mOffset;
	return imagePosition / mScale;
}

Vector2f ftl::gui2::ImageView::clampedImageCoordinateAt(const Vector2f& position) const {
	auto imageCoordinate = imageCoordinateAt(position);
	return imageCoordinate.cwiseMax(Vector2f::Zero()).cwiseMin(imageSizeF());
}

Vector2f ftl::gui2::ImageView::positionForCoordinate(const Vector2f& imageCoordinate) const {
	return mScale*imageCoordinate + mOffset;
}

void ftl::gui2::ImageView::setImageCoordinateAt(const Vector2f& position, const Vector2f& imageCoordinate) {
	// Calculate where the new offset must be in order to satisfy the image position equation.
	// Round the floating point values to balance out the floating point to integer conversions.
	mOffset = position - (imageCoordinate * mScale);

	// Clamp offset so that the image remains near the screen.
	mOffset = mOffset.cwiseMin(sizeF()).cwiseMax(-scaledImageSizeF());
}

void ftl::gui2::ImageView::center() {
	mOffset = (sizeF() - scaledImageSizeF()) / 2;
}

void ftl::gui2::ImageView::fit() {
	// Calculate the appropriate scaling factor.
	mScale = (sizeF().cwiseQuotient(imageSizeF())).minCoeff();
	center();
}

void ftl::gui2::ImageView::setScaleCentered(float scale) {
	auto centerPosition = sizeF() / 2;
	auto p = imageCoordinateAt(centerPosition);
	mScale = scale;
	setImageCoordinateAt(centerPosition, p);
}

void ftl::gui2::ImageView::moveOffset(const Vector2f& delta) {
	// Apply the delta to the offset.
	mOffset += delta;

	// Prevent the image from going out of bounds.
	auto scaledSize = scaledImageSizeF();
	if (mOffset.x() + scaledSize.x() < 0)
		mOffset.x() = -scaledSize.x();
	if (mOffset.x() > sizeF().x())
		mOffset.x() = sizeF().x();
	if (mOffset.y() + scaledSize.y() < 0)
		mOffset.y() = -scaledSize.y();
	if (mOffset.y() > sizeF().y())
		mOffset.y() = sizeF().y();
}

void ftl::gui2::ImageView::zoom(int amount, const Vector2f& focusPosition) {
	auto focusedCoordinate = imageCoordinateAt(focusPosition);
	float scaleFactor = std::pow(mZoomSensitivity, amount);
	mScale = std::max(0.01f, scaleFactor * mScale);
	setImageCoordinateAt(focusPosition, focusedCoordinate);
}

bool ftl::gui2::ImageView::mouseDragEvent(const Vector2i& p, const Vector2i& rel, int button, int /*modifiers*/) {
	if ((button & (1 << GLFW_MOUSE_BUTTON_RIGHT)) != 0 && !mFixedOffset) {
		setImageCoordinateAt((p + rel).cast<float>(), imageCoordinateAt(p.cast<float>()));
		return true;
	}
	return false;
}

bool ftl::gui2::ImageView::gridVisible() const {
	return (mGridThreshold != -1) && (mScale > mGridThreshold);
}

bool ftl::gui2::ImageView::pixelInfoVisible() const {
	return mPixelInfoCallback && (mPixelInfoThreshold != -1) && (mScale > mPixelInfoThreshold);
}

bool ftl::gui2::ImageView::helpersVisible() const {
	return gridVisible() || pixelInfoVisible();
}

bool ftl::gui2::ImageView::scrollEvent(const Vector2i& p, const Vector2f& rel) {
	if (mFixedScale)
		return false;
	float v = rel.y();
	if (std::abs(v) < 1)
		v = std::copysign(1.f, v);
	zoom(v, (p - position()).cast<float>());
	return true;
}

bool ftl::gui2::ImageView::keyboardEvent(int key, int /*scancode*/, int action, int modifiers) {
	if (action) {
		switch (key) {
		case GLFW_KEY_LEFT:
			if (!mFixedOffset) {
				if (GLFW_MOD_CONTROL & modifiers)
					moveOffset(Vector2f(30, 0));
				else
					moveOffset(Vector2f(10, 0));
				return true;
			}
			break;
		case GLFW_KEY_RIGHT:
			if (!mFixedOffset) {
				if (GLFW_MOD_CONTROL & modifiers)
					moveOffset(Vector2f(-30, 0));
				else
					moveOffset(Vector2f(-10, 0));
				return true;
			}
			break;
		case GLFW_KEY_DOWN:
			if (!mFixedOffset) {
				if (GLFW_MOD_CONTROL & modifiers)
					moveOffset(Vector2f(0, -30));
				else
					moveOffset(Vector2f(0, -10));
				return true;
			}
			break;
		case GLFW_KEY_UP:
			if (!mFixedOffset) {
				if (GLFW_MOD_CONTROL & modifiers)
					moveOffset(Vector2f(0, 30));
				else
					moveOffset(Vector2f(0, 10));
				return true;
			}
			break;
		}
	}
	return false;
}

bool ftl::gui2::ImageView::keyboardCharacterEvent(unsigned int codepoint) {
	switch (codepoint) {
	case '-':
		if (!mFixedScale) {
			zoom(-1, sizeF() / 2);
			return true;
		}
		break;
	case '+':
		if (!mFixedScale) {
			zoom(1, sizeF() / 2);
			return true;
		}
		break;
	case 'c':
		if (!mFixedOffset) {
			center();
			return true;
		}
		break;
	case 'f':
		if (!mFixedOffset && !mFixedScale) {
			fit();
			return true;
		}
		break;
	case '1': case '2': case '3': case '4': case '5':
	case '6': case '7': case '8': case '9':
		if (!mFixedScale) {
			setScaleCentered(1 << (codepoint - '1'));
			return true;
		}
		break;
	default:
		return false;
	}
	return false;
}

Vector2i ftl::gui2::ImageView::preferredSize(NVGcontext* /*ctx*/) const {
	return mImageSize;
}

void ftl::gui2::ImageView::performLayout(NVGcontext* ctx) {
	Widget::performLayout(ctx);
}

void ftl::gui2::ImageView::draw(NVGcontext* ctx) {
	Widget::draw(ctx);

	if (mImageID != unsigned(-1)) {
		nvgEndFrame(ctx); // Flush the NanoVG draw stack, not necessary to call nvgBeginFrame afterwards.
		//drawImageBorder(ctx);

		// Calculate several variables that need to be send to OpenGL in order for the image to be
		// properly displayed inside the widget.
		const Screen* screen = dynamic_cast<const Screen*>(this->screen());
		Vector2f screenSize = screen->size().cast<float>();
		Vector2f scaleFactor = mScale * imageSizeF().cwiseQuotient(screenSize);
		Vector2f positionInScreen = absolutePosition().cast<float>();
		Vector2f positionAfterOffset = positionInScreen + mOffset;
		Vector2f imagePosition = positionAfterOffset.cwiseQuotient(screenSize);
		glEnable(GL_SCISSOR_TEST);
		float r = screen->pixelRatio();
		glScissor(positionInScreen.x() * r,
				(screenSize.y() - positionInScreen.y() - size().y()) * r,
				size().x() * r, size().y() * r);
		mShader.bind();
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, mImageID);
		mShader.setUniform("image", 0);
		mShader.setUniform("scaleFactor", scaleFactor);
		mShader.setUniform("position", imagePosition);
		mShader.drawIndexed(GL_TRIANGLES, 0, 2);
		glDisable(GL_SCISSOR_TEST);
	}

	if (helpersVisible())
		drawHelpers(ctx);

	//drawWidgetBorder(ctx);
}

void ftl::gui2::ImageView::updateImageParameters() {
	// Query the width of the OpenGL texture.
	glBindTexture(GL_TEXTURE_2D, mImageID);
	GLint w, h;
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &w);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &h);
	mImageSize = Vector2i(w, h);
}

void ftl::gui2::ImageView::drawWidgetBorder(NVGcontext* ctx) const {
	nvgBeginPath(ctx);
	nvgStrokeWidth(ctx, 1);
	nvgRoundedRect(ctx, mPos.x() + 0.5f, mPos.y() + 0.5f, mSize.x() - 1,
				   mSize.y() - 1, 0);
	nvgStrokeColor(ctx, mTheme->mWindowPopup);
	nvgStroke(ctx);

	nvgBeginPath(ctx);
	nvgRoundedRect(ctx, mPos.x() + 0.5f, mPos.y() + 0.5f, mSize.x() - 1,
				   mSize.y() - 1, mTheme->mButtonCornerRadius);
	nvgStrokeColor(ctx, mTheme->mBorderDark);
	nvgStroke(ctx);
}

void ftl::gui2::ImageView::drawImageBorder(NVGcontext* ctx) const {
	nvgSave(ctx);
	nvgBeginPath(ctx);
	nvgScissor(ctx, mPos.x(), mPos.y(), mSize.x(), mSize.y());
	nvgStrokeWidth(ctx, 1.0f);
	Vector2i borderPosition = mPos + mOffset.cast<int>();
	Vector2i borderSize = scaledImageSizeF().cast<int>();
	nvgRect(ctx, borderPosition.x() - 0.5f, borderPosition.y() - 0.5f,
			borderSize.x() + 1, borderSize.y() + 1);
	nvgStrokeColor(ctx, Color(1.0f, 1.0f, 1.0f, 1.0f));
	nvgStroke(ctx);
	nvgResetScissor(ctx);
	nvgRestore(ctx);
}

void ftl::gui2::ImageView::drawHelpers(NVGcontext* ctx) const {
	// We need to apply mPos after the transformation to account for the position of the widget
	// relative to the parent.
	Vector2f upperLeftCorner = positionForCoordinate(Vector2f::Zero()) + positionF();
	Vector2f lowerRightCorner = positionForCoordinate(imageSizeF()) + positionF();
	if (gridVisible())
		drawPixelGrid(ctx, upperLeftCorner, lowerRightCorner, mScale);
	if (pixelInfoVisible())
		drawPixelInfo(ctx, mScale);
}

void ftl::gui2::ImageView::drawPixelGrid(NVGcontext* ctx, const Vector2f& upperLeftCorner,
							  const Vector2f& lowerRightCorner, float stride) {
	nvgBeginPath(ctx);

	// Draw the vertical grid lines
	float currentX = upperLeftCorner.x();
	while (currentX <= lowerRightCorner.x()) {
		nvgMoveTo(ctx, std::round(currentX), std::round(upperLeftCorner.y()));
		nvgLineTo(ctx, std::round(currentX), std::round(lowerRightCorner.y()));
		currentX += stride;
	}

	// Draw the horizontal grid lines
	float currentY = upperLeftCorner.y();
	while (currentY <= lowerRightCorner.y()) {
		nvgMoveTo(ctx, std::round(upperLeftCorner.x()), std::round(currentY));
		nvgLineTo(ctx, std::round(lowerRightCorner.x()), std::round(currentY));
		currentY += stride;
	}

	nvgStrokeWidth(ctx, 1.0f);
	nvgStrokeColor(ctx, Color(1.0f, 1.0f, 1.0f, 0.2f));
	nvgStroke(ctx);
}

void ftl::gui2::ImageView::drawPixelInfo(NVGcontext* ctx, float stride) const {
	// Extract the image coordinates at the two corners of the widget.
	Vector2i topLeft = clampedImageCoordinateAt(Vector2f::Zero())
						   .unaryExpr([](float x) { return std::floor(x); })
						   .cast<int>();

	Vector2i bottomRight = clampedImageCoordinateAt(sizeF())
							   .unaryExpr([](float x) { return std::ceil(x); })
							   .cast<int>();

	// Extract the positions for where to draw the text.
	Vector2f currentCellPosition =
		(positionF() + positionForCoordinate(topLeft.cast<float>()));

	float xInitialPosition = currentCellPosition.x();
	int xInitialIndex = topLeft.x();

	// Properly scale the pixel information for the given stride.
	auto fontSize = stride * mFontScaleFactor;
	static constexpr float maxFontSize = 30.0f;
	fontSize = fontSize > maxFontSize ? maxFontSize : fontSize;
	nvgBeginPath(ctx);
	nvgFontSize(ctx, fontSize);
	nvgTextAlign(ctx, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
	nvgFontFace(ctx, "sans");
	while (topLeft.y() != bottomRight.y()) {
		while (topLeft.x() != bottomRight.x()) {
			writePixelInfo(ctx, currentCellPosition, topLeft, stride, fontSize);
			currentCellPosition.x() += stride;
			++topLeft.x();
		}
		currentCellPosition.x() = xInitialPosition;
		currentCellPosition.y() += stride;
		++topLeft.y();
		topLeft.x() = xInitialIndex;
	}
}

void ftl::gui2::ImageView::writePixelInfo(NVGcontext* ctx, const Vector2f& cellPosition,
							   const Vector2i& pixel, float stride, float fontSize) const {
	auto pixelData = mPixelInfoCallback(pixel);
	auto pixelDataRows = tokenize(pixelData.first);

	// If no data is provided for this pixel then simply return.
	if (pixelDataRows.empty())
		return;

	nvgFillColor(ctx, pixelData.second);
	float yOffset = (stride - fontSize * pixelDataRows.size()) / 2;
	for (size_t i = 0; i != pixelDataRows.size(); ++i) {
		nvgText(ctx, cellPosition.x() + stride / 2, cellPosition.y() + yOffset,
				pixelDataRows[i].data(), nullptr);
		yOffset += fontSize;
	}
}

////////////////////////////////////////////////////////////////////////////////

FTLImageView::~FTLImageView() {
}

void FTLImageView::draw(NVGcontext* ctx) {
	if (texture_.isValid()) {
		if (!was_valid_) { fit(); }
		ImageView::draw(ctx);
	}
	was_valid_ = texture_.isValid();
}

GLTexture& FTLImageView::texture() {
	return texture_;
}

void FTLImageView::copyFrom(const ftl::cuda::TextureObject<uchar4> &buf, cudaStream_t stream ) {
	texture_.copyFrom(buf, stream);
	bindImage(texture_.texture());
}

void FTLImageView::copyFrom(const cv::Mat &im, cudaStream_t stream) {
	texture_.copyFrom(im, stream);
	bindImage(texture_.texture());
}

void FTLImageView::copyFrom(const cv::cuda::GpuMat &im, cudaStream_t stream) {
	texture_.copyFrom(im, stream);
	bindImage(texture_.texture());
}

void FTLImageView::copyFrom(ftl::rgbd::Frame& frame, ftl::codecs::Channel channel) {
	if (frame.hasOpenGL(channel)) {
		bindImage(frame.getOpenGL(channel));
		if (texture_.isValid()) {
			texture_.free();
		}
	}
	else if (frame.isGPU(channel)) {
		copyFrom(frame.get<cv::cuda::GpuMat>(channel));
	}
	else {
		copyFrom(frame.get<cv::Mat>(channel));
	}
}

nanogui::Vector2i ftl::gui2::FTLImageView::preferredSize(NVGcontext* /*ctx*/) const {
	/** this avoids issues if layout not set to fill/maximum */
	return mSize;
}

// ==== StereoImageView ========================================================

using ftl::gui2::StereoImageView;

StereoImageView::StereoImageView(nanogui::Widget* parent, nanogui::Orientation orientation) :
		nanogui::Widget(parent), orientation_(orientation) {

	setLayout(new nanogui::BoxLayout(orientation_, nanogui::Alignment::Fill));

	left_ = new FTLImageView(this);
	right_ = new FTLImageView(this);

	// disables mouse/keyboard events in widgets
	left_->setFixedOffset(true);
	left_->setFixedScale(true);
	right_->setFixedOffset(true);
	right_->setFixedScale(true);
}

bool StereoImageView::mouseMotionEvent(const nanogui::Vector2i &p, const nanogui::Vector2i &rel, int button, int modifiers) {
	if ((button & (1 << GLFW_MOUSE_BUTTON_RIGHT)) != 0) {
		nanogui::Vector2f posl = left_->imageCoordinateAt(p.cast<float>());
		nanogui::Vector2f posr = right_->imageCoordinateAt(p.cast<float>());
		if (posl.minCoeff() > 0) {
			left_->setImageCoordinateAt((p + rel).cast<float>(), posl);
			right_->setImageCoordinateAt((p + rel).cast<float>(), posl);
		}
		if (posr.minCoeff() > 0) {
			left_->setImageCoordinateAt((p + rel).cast<float>(), posr);
			right_->setImageCoordinateAt((p + rel).cast<float>(), posr);
		}
		return true;
	}
	return false;
}

bool StereoImageView::scrollEvent(const nanogui::Vector2i& p, const nanogui::Vector2f& rel) {
	// synchronized zoom

	float v = rel.y();

	// zooming on right image?
	bool zoom_right =
		((p.x() >= left_->absolutePosition().x()) && (orientation_ == nanogui::Orientation::Horizontal)) ||
		((p.y() >= left_->absolutePosition().y()) && (orientation_ == nanogui::Orientation::Vertical));

	if (orientation_ == nanogui::Orientation::Horizontal) {
		if (zoom_right) {
			left_->zoom(v, (p - nanogui::Vector2i{left_->width(), 0} - left_->position()).cast<float>());
			right_->zoom(v, (p - right_->position()).cast<float>());
		}
		else {
			left_->zoom(v, (p - left_->position()).cast<float>());
			right_->zoom(v, (nanogui::Vector2i{right_->width(), 0} + p - right_->position()).cast<float>());
		}
	}
	else { // same as above, flip x/y
		if (zoom_right) {
			left_->zoom(v, (p - nanogui::Vector2i{0, left_->height()} - left_->position()).cast<float>());
			right_->zoom(v, (p - right_->position()).cast<float>());
		}
		else {
			left_->zoom(v, (p - left_->position()).cast<float>());
			right_->zoom(v, (nanogui::Vector2i{0, right_->height()} + p - right_->position()).cast<float>());
		}
	}
	return true;
}

bool StereoImageView::keyboardEvent(int key, int /*scancode*/, int action, int modifiers) {
	return true; // copy code from above (ImageView)?
}


void StereoImageView::fit() {
	left()->fit();
	right()->fit();
}

bool StereoImageView::keyboardCharacterEvent(unsigned int codepoint) {
	switch (codepoint) {
	case 'c':
		left_->center();
		right_->center();
		return true;

	case 'f':
		left_->fit();
		right_->fit();
		return true;

	default:
		return true;
	}
}

void StereoImageView::performLayout(NVGcontext *ctx) {
	if (orientation_ == nanogui::Orientation::Horizontal) {
		left_->setSize({width()/2, height()});
		right_->setSize({width()/2, height()});
	}
	else { // Orientation::Vertical
		left_->setSize({width(), height()/2});
		right_->setSize({width(), height()/2});
	}
	Widget::performLayout(ctx);
}
