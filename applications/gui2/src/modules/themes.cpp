#include "themes.hpp"
#include "nanogui/theme.h"
#include "../screen.hpp"

using ftl::gui2::Themes;
using nanogui::Theme;

void Themes::init() {
	auto* toolbuttheme = screen->getTheme("toolbutton");
	toolbuttheme->mBorderDark = nanogui::Color(0,0);
	toolbuttheme->mBorderLight = nanogui::Color(0,0);
	toolbuttheme->mButtonGradientBotFocused = nanogui::Color(60,255);
	toolbuttheme->mButtonGradientBotUnfocused = nanogui::Color(0,0);
	toolbuttheme->mButtonGradientTopFocused = nanogui::Color(60,255);
	toolbuttheme->mButtonGradientTopUnfocused = nanogui::Color(0,0);
	toolbuttheme->mButtonGradientTopPushed = nanogui::Color(60,180);
	toolbuttheme->mButtonGradientBotPushed = nanogui::Color(60,180);
	toolbuttheme->mTextColor = nanogui::Color(0.9f,0.9f,0.9f,0.9f);
	toolbuttheme->mWindowDropShadowSize = 0;
	toolbuttheme->mDropShadow = nanogui::Color(0,0);

	auto* windowtheme = screen->getTheme("window_light");
	windowtheme->mWindowFillFocused = nanogui::Color(220, 200);
	windowtheme->mWindowFillUnfocused = nanogui::Color(220, 200);
	windowtheme->mWindowHeaderGradientBot = nanogui::Color(60,230);
	windowtheme->mWindowHeaderGradientTop = nanogui::Color(60,230);
	windowtheme->mWindowHeaderSepBot = nanogui::Color(60, 230);
	windowtheme->mTextColor = nanogui::Color(20,255);
	windowtheme->mDisabledTextColor = nanogui::Color(140, 255);
	windowtheme->mWindowCornerRadius = 2;
	windowtheme->mButtonGradientBotFocused = nanogui::Color(210,255);
	windowtheme->mButtonGradientBotUnfocused = nanogui::Color(190,255);
	windowtheme->mButtonGradientTopFocused = nanogui::Color(230,255);
	windowtheme->mButtonGradientTopUnfocused = nanogui::Color(230,255);
	windowtheme->mButtonGradientTopPushed = nanogui::Color(170,255);
	windowtheme->mButtonGradientBotPushed = nanogui::Color(210,255);
	windowtheme->mBorderDark = nanogui::Color(150,255);
	windowtheme->mBorderMedium = nanogui::Color(165,255);
	windowtheme->mBorderLight = nanogui::Color(230,255);
	windowtheme->mButtonFontSize = 16;
	windowtheme->mTextColorShadow = nanogui::Color(0,0);
	windowtheme->mWindowTitleUnfocused = windowtheme->mWindowTitleFocused;
	windowtheme->mWindowTitleFocused = nanogui::Color(240,255);
	windowtheme->mIconScale = 0.85f;

	auto* viewtheme = screen->getTheme("view");
	viewtheme->mWindowFillFocused = nanogui::Color(0, 0);
	viewtheme->mWindowFillUnfocused = nanogui::Color(0, 0);
	viewtheme->mWindowCornerRadius = 0;
	viewtheme->mBorderDark = nanogui::Color(0 ,0);
	viewtheme->mBorderMedium = nanogui::Color(0 ,0);
	viewtheme->mBorderLight = nanogui::Color(0 ,0);
	viewtheme->mWindowHeaderGradientBot = nanogui::Color(0, 0);
	viewtheme->mWindowHeaderGradientTop = nanogui::Color(0, 0);
	viewtheme->mWindowHeaderSepBot = nanogui::Color(0, 0);
	viewtheme->mTextColorShadow = nanogui::Color(0, 0);
	viewtheme->mWindowDropShadowSize = 0;

	auto* windowtheme_dark = screen->getTheme("window_dark");
	windowtheme_dark->mWindowCornerRadius = 5;
	/*windowtheme_dark->mButtonGradientBotFocused = nanogui::Color(90,255);
	windowtheme_dark->mButtonGradientBotUnfocused = nanogui::Color(70,255);
	windowtheme_dark->mButtonGradientTopFocused = nanogui::Color(110,255);
	windowtheme_dark->mButtonGradientTopUnfocused = nanogui::Color(110,255);
	windowtheme_dark->mButtonGradientTopPushed = nanogui::Color(50,255);
	windowtheme_dark->mButtonGradientBotPushed = nanogui::Color(90,255);*/
	windowtheme_dark->mButtonGradientBotFocused = nanogui::Color(60,255);
	windowtheme_dark->mButtonGradientBotUnfocused = nanogui::Color(35,35,40,180);
	windowtheme_dark->mButtonGradientTopFocused = nanogui::Color(60,255);
	windowtheme_dark->mButtonGradientTopUnfocused = nanogui::Color(35,35,40,180);
	windowtheme_dark->mButtonGradientTopPushed = nanogui::Color(90,180);
	windowtheme_dark->mButtonGradientBotPushed = nanogui::Color(90,180);
	windowtheme_dark->mButtonFontSize = 16;
	windowtheme_dark->mIconScale = 0.85f;
	windowtheme_dark->mBorderDark = nanogui::Color(20,0);
	windowtheme_dark->mBorderMedium = nanogui::Color(20,0);
	windowtheme_dark->mBorderLight = nanogui::Color(20,0);

	auto* mediatheme = screen->getTheme("media");
	mediatheme->mIconScale = 1.2f;
	mediatheme->mWindowDropShadowSize = 0;
	mediatheme->mWindowFillFocused = nanogui::Color(45, 150);
	mediatheme->mWindowFillUnfocused = nanogui::Color(45, 80);
	mediatheme->mButtonGradientTopUnfocused = nanogui::Color(0,0);
	mediatheme->mButtonGradientBotUnfocused = nanogui::Color(0,0);
	mediatheme->mButtonGradientTopFocused = nanogui::Color(80,230);
	mediatheme->mButtonGradientBotFocused = nanogui::Color(80,230);
	mediatheme->mIconColor = nanogui::Color(255,255);
	mediatheme->mTextColor = nanogui::Color(1.0f,1.0f,1.0f,1.0f);
	mediatheme->mBorderDark = nanogui::Color(0,0);
	mediatheme->mBorderMedium = nanogui::Color(0,0);
	mediatheme->mBorderLight = nanogui::Color(0,0);
	mediatheme->mDropShadow = nanogui::Color(0,0);
	mediatheme->mButtonFontSize = 30;
	mediatheme->mStandardFontSize = 20;

	// https://flatuicolors.com/palette/defo
	screen->setColor("highlight1", nanogui::Color(231, 76, 60, 255)); // red
	screen->setColor("highlight2", nanogui::Color(52, 152, 219, 255)); // blue

	screen->setColor("highlight1_disabled", nanogui::Color(166, 166, 166, 255));
	screen->setColor("highlight2_disabled", nanogui::Color(166, 166, 166, 255));
}
