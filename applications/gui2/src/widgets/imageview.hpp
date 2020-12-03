/**
 * @file imageview.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Sebastian Hahta
 */

/*
	nanogui/imageview.h -- Widget used to display images.

	The image view widget was contributed by Stefan Ivanov.

	NanoGUI was developed by Wenzel Jakob <wenzel.jakob@epfl.ch>.
	The widget drawing code is based on the NanoVG demo application
	by Mikko Mononen.

	All rights reserved. Use of this source code is governed by a
	BSD-style license that can be found in the LICENSE.txt file.
*/
/** \file */

#pragma once

#include <nanogui/widget.h>
#include <nanogui/glutil.h>
#include <nanogui/layout.h>
#include <functional>

#include <ftl/rgbd/frame.hpp>
#include <ftl/codecs/channels.hpp>
#include <ftl/utility/gltexture.hpp>

namespace ftl
{
namespace gui2 {


/**
 * \class ImageView imageview.h nanogui/imageview.h
 *
 * \brief Widget used to display images.
 */
class NANOGUI_EXPORT ImageView : public nanogui::Widget {
public:
	ImageView(nanogui::Widget* parent, GLuint imageID = -1);
	virtual ~ImageView();

	void bindImage(GLuint imageId);

	nanogui::GLShader& imageShader() { return mShader; }

	nanogui::Vector2f positionF() const { return mPos.cast<float>(); }
	nanogui::Vector2f sizeF() const { return mSize.cast<float>(); }

	const nanogui::Vector2i& imageSize() const { return mImageSize; }
	nanogui::Vector2i scaledImageSize() const { return (mScale * mImageSize.cast<float>()).cast<int>(); }
	nanogui::Vector2f imageSizeF() const { return mImageSize.cast<float>(); }
	nanogui::Vector2f scaledImageSizeF() const { return (mScale * mImageSize.cast<float>()); }

	const nanogui::Vector2f& offset() const { return mOffset; }
	void setOffset(const nanogui::Vector2f& offset) { mOffset = offset; }
	float scale() const { return mScale; }
	void setScale(float scale) { mScale = scale > 0.01f ? scale : 0.01f; }

	inline void setFlipped(bool flipped) { flipped_ = flipped; }

	bool fixedOffset() const { return mFixedOffset; }
	void setFixedOffset(bool fixedOffset) { mFixedOffset = fixedOffset; }
	bool fixedScale() const { return mFixedScale; }
	void setFixedScale(bool fixedScale) { mFixedScale = fixedScale; }

	float zoomSensitivity() const { return mZoomSensitivity; }
	void setZoomSensitivity(float zoomSensitivity) { mZoomSensitivity = zoomSensitivity; }

	float gridThreshold() const { return mGridThreshold; }
	void setGridThreshold(float gridThreshold) { mGridThreshold = gridThreshold; }

	float pixelInfoThreshold() const { return mPixelInfoThreshold; }
	void setPixelInfoThreshold(float pixelInfoThreshold) { mPixelInfoThreshold = pixelInfoThreshold; }

#ifndef DOXYGEN_SHOULD_SKIP_THIS
	void setPixelInfoCallback(const std::function<std::pair<std::string, nanogui::Color>(const nanogui::Vector2i&)>& callback) {
		mPixelInfoCallback = callback;
	}
	const std::function<std::pair<std::string, nanogui::Color>(const nanogui::Vector2i&)>& pixelInfoCallback() const {
		return mPixelInfoCallback;
	}
#endif // DOXYGEN_SHOULD_SKIP_THIS

	void setFontScaleFactor(float fontScaleFactor) { mFontScaleFactor = fontScaleFactor; }
	float fontScaleFactor() const { return mFontScaleFactor; }

	// Image transformation functions.

	/// Calculates the image coordinates of the given pixel position on the widget.
	nanogui::Vector2f imageCoordinateAt(const nanogui::Vector2f& position) const;

	/**
	 * Calculates the image coordinates of the given pixel position on the widget.
	 * If the position provided corresponds to a coordinate outside the range of
	 * the image, the coordinates are clamped to edges of the image.
	 */
	nanogui::Vector2f clampedImageCoordinateAt(const nanogui::Vector2f& position) const;

	/// Calculates the position inside the widget for the given image coordinate. Origin?
	nanogui::Vector2f positionForCoordinate(const nanogui::Vector2f& imageCoordinate) const;

	/**
	 * Modifies the internal state of the image viewer widget so that the pixel at the provided
	 * position on the widget has the specified image coordinate. Also clamps the values of offset
	 * to the sides of the widget.
	 */
	void setImageCoordinateAt(const nanogui::Vector2f& position, const nanogui::Vector2f& imageCoordinate);

	/// Centers the image without affecting the scaling factor.
	void center();

	/// Centers and scales the image so that it fits inside the widgets.
	void fit();

	/// Set the scale while keeping the image centered
	void setScaleCentered(float scale);

	/// Moves the offset by the specified amount. Does bound checking.
	void moveOffset(const nanogui::Vector2f& delta);

	/**
	 * Changes the scale factor by the provided amount modified by the zoom sensitivity member variable.
	 * The scaling occurs such that the image coordinate under the focused position remains in
	 * the same position before and after the scaling.
	 */
	void zoom(int amount, const nanogui::Vector2f& focusPosition);

	bool keyboardEvent(int key, int scancode, int action, int modifiers) override;
	bool keyboardCharacterEvent(unsigned int codepoint) override;

	//bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) override;
	bool mouseDragEvent(const nanogui::Vector2i &p, const nanogui::Vector2i &rel, int button, int modifiers) override;
	bool scrollEvent(const nanogui::Vector2i &p, const nanogui::Vector2f &rel) override;

	/// Function indicating whether the grid is currently visible.
	bool gridVisible() const;

	/// Function indicating whether the pixel information is currently visible.
	bool pixelInfoVisible() const;

	/// Function indicating whether any of the overlays are visible.
	bool helpersVisible() const;

	nanogui::Vector2i preferredSize(NVGcontext* ctx) const override;
	void performLayout(NVGcontext* ctx) override;
	void draw(NVGcontext* ctx) override;

protected:
	// Helper image methods.
	void updateImageParameters();

	// Helper drawing methods.
	void drawWidgetBorder(NVGcontext* ctx) const;
	void drawImageBorder(NVGcontext* ctx) const;
	void drawHelpers(NVGcontext* ctx) const;
	static void drawPixelGrid(NVGcontext* ctx, const nanogui::Vector2f& upperLeftCorner,
							  const nanogui::Vector2f& lowerRightCorner, float stride);
	void drawPixelInfo(NVGcontext* ctx, float stride) const;
	void writePixelInfo(NVGcontext* ctx, const nanogui::Vector2f& cellPosition,
						const nanogui::Vector2i& pixel, float stride, float fontSize) const;

	// Image parameters.
	nanogui::GLShader mShader;
	GLuint mImageID;
	nanogui::Vector2i mImageSize;

	// Image display parameters.
	float mScale;
	nanogui::Vector2f mOffset;
	bool mFixedScale;
	bool mFixedOffset;
	bool flipped_ = false;

	// Fine-tuning parameters.
	float mZoomSensitivity = 1.1f;

	// Image info parameters.
	float mGridThreshold = -1;
	float mPixelInfoThreshold = -1;

	// Image pixel data display members.
	std::function<std::pair<std::string, nanogui::Color>(const nanogui::Vector2i&)> mPixelInfoCallback;
	float mFontScaleFactor = 0.2f;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Simple wrapper for drawing FTLImageView.
 */
class FTLImageView : public ImageView {
public:
	using ImageView::ImageView;

	FTLImageView(nanogui::Widget* parent, GLuint imageID = -1) : ImageView(parent, imageID), was_valid_(false) {}
	virtual ~FTLImageView();

	virtual void draw(NVGcontext* ctx) override;
	virtual nanogui::Vector2i preferredSize(NVGcontext* ctx) const override;

	/** Get GLTexture instance */
	ftl::utility::GLTexture& texture();

	/** Copy&Bind */
	void copyFrom(const ftl::cuda::TextureObject<uchar4> &buf, cudaStream_t stream = cudaStreamDefault);
	void copyFrom(const cv::Mat &im, cudaStream_t stream = cudaStreamDefault);
	void copyFrom(const cv::cuda::GpuMat &im, cudaStream_t stream = cudaStreamDefault);

	/** From frame, use OpenGL if available (no copy), otherwise copy from GPU/CPU */
	void copyFrom(ftl::rgbd::Frame& frame, ftl::codecs::Channel channel);

private:
	ftl::utility::GLTexture texture_;
	bool was_valid_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** Two ImageViews with synchronized zoom and pan. Widget split in two equal
 * size sections (left and right). With vertical orientation right is the lower
 * image.
*/
class StereoImageView : public nanogui::Widget {
public:
	StereoImageView(nanogui::Widget* parent, nanogui::Orientation orientation = nanogui::Orientation::Horizontal);

	virtual void performLayout(NVGcontext* ctx) override;

	bool keyboardEvent(int key, int scancode, int action, int modifiers) override;
	bool keyboardCharacterEvent(unsigned int codepoint) override;
	bool mouseMotionEvent(const nanogui::Vector2i &p, const nanogui::Vector2i &rel, int button, int modifiers) override;
	bool scrollEvent(const nanogui::Vector2i &p, const nanogui::Vector2f &rel) override;

	FTLImageView* left() { return left_; }
	FTLImageView* right() { return right_; }

	/** get image coordinate at given widget coordinate */
	nanogui::Vector2f imageCoordinateAt(const nanogui::Vector2f& position) const;

	nanogui::Orientation orientation() { return orientation_; }

	void fit();

	void bindLeft(GLuint id) { left_->texture().free(); left_->bindImage(id); }
	void bindRight(GLuint id) { right_->texture().free(); right_->bindImage(id); }

private:
	nanogui::Orientation orientation_;
	FTLImageView* left_;
	FTLImageView* right_;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}
