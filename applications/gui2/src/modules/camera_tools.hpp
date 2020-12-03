/**
 * @file camera_tools.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_GUI_CAMERA_TOOLS_HPP_
#define _FTL_GUI_CAMERA_TOOLS_HPP_

namespace ftl {
namespace gui2 {

enum class Tools {
	NONE,
	SELECT_POINT,			// Touch 2D
	MOVEMENT,				// 3D first person camera controls
	PAN,					// 2D Panning
	CENTRE_VIEW,
	ZOOM_FIT,
	ZOOM_IN,
	ZOOM_OUT,
	CLIPPING,
	OVERLAY,
	LAYOUT,
	MOVE_CURSOR,			// Move 3D Cursor
	ROTATE_CURSOR,
	ORIGIN_TO_CURSOR,
	RESET_ORIGIN,
	SAVE_CURSOR,
	ROTATE_X,
	ROTATE_Y,
	ROTATE_Z,
	TRANSLATE_X,
	TRANSLATE_Y,
	TRANSLATE_Z,
	INSPECT_POINT
};

enum class ToolGroup {
	MOUSE_MOTION,
	VIEW_2D_ACTIONS,
	VIEW_3D_LAYERS,
	VIEW_3D_ACTIONS
};

}
}

#endif