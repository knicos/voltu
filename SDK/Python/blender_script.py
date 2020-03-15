import bpy
import numpy as np
from mathutils import Matrix, Vector

_d_max = 65504.0

################################################################################
# https://blender.stackexchange.com/a/120063

# BKE_camera_sensor_size
def get_sensor_size(sensor_fit, sensor_x, sensor_y):
    if sensor_fit == 'VERTICAL':
        return sensor_y
    return sensor_x

# BKE_camera_sensor_fit
def get_sensor_fit(sensor_fit, size_x, size_y):
    if sensor_fit == 'AUTO':
        if size_x >= size_y:
            return 'HORIZONTAL'
        else:
            return 'VERTICAL'
    return sensor_fit

# Build intrinsic camera parameters from Blender camera data
#
# See notes on this in 
# blender.stackexchange.com/questions/15102/what-is-blenders-camera-projection-matrix-model
# as well as
# https://blender.stackexchange.com/a/120063/3581
def get_calibration_matrix_K_from_blender(camd):
    if camd.type != 'PERSP':
        raise ValueError('Non-perspective cameras not supported')
    scene = bpy.context.scene
    f_in_mm = camd.lens
    scale = scene.render.resolution_percentage / 100
    resolution_x_in_px = scale * scene.render.resolution_x
    resolution_y_in_px = scale * scene.render.resolution_y
    sensor_size_in_mm = get_sensor_size(camd.sensor_fit, camd.sensor_width, camd.sensor_height)
    sensor_fit = get_sensor_fit(
        camd.sensor_fit,
        scene.render.pixel_aspect_x * resolution_x_in_px,
        scene.render.pixel_aspect_y * resolution_y_in_px
    )
    pixel_aspect_ratio = scene.render.pixel_aspect_y / scene.render.pixel_aspect_x
    if sensor_fit == 'HORIZONTAL':
        view_fac_in_px = resolution_x_in_px
    else:
        view_fac_in_px = pixel_aspect_ratio * resolution_y_in_px
    pixel_size_mm_per_px = sensor_size_in_mm / f_in_mm / view_fac_in_px
    s_u = 1 / pixel_size_mm_per_px
    s_v = 1 / pixel_size_mm_per_px / pixel_aspect_ratio

    # Parameters of intrinsic calibration matrix K
    u_0 = resolution_x_in_px / 2 - camd.shift_x * view_fac_in_px
    v_0 = resolution_y_in_px / 2 + camd.shift_y * view_fac_in_px / pixel_aspect_ratio
    skew = 0 # only use rectangular pixels

    K = Matrix(
        ((s_u, skew, u_0),
        (   0,  s_v, v_0),
        (   0,    0,   1)))
    return K

# Returns camera rotation and translation matrices from Blender.
# 
# There are 3 coordinate systems involved:
#    1. The World coordinates: "world"
#       - right-handed
#    2. The Blender camera coordinates: "bcam"
#       - x is horizontal
#       - y is up
#       - right-handed: negative z look-at direction
#    3. The desired computer vision camera coordinates: "cv"
#       - x is horizontal
#       - y is down (to align to the actual pixel coordinates 
#         used in digital images)
#       - right-handed: positive z look-at direction
def get_3x4_RT_matrix_from_blender(cam):
    # bcam stands for blender camera
    R_bcam2cv = Matrix(
        ((1, 0,  0),
        (0, -1, 0),
        (0, 0, -1)))

    # Transpose since the rotation is object rotation, 
    # and we want coordinate rotation
    # R_world2bcam = cam.rotation_euler.to_matrix().transposed()
    # T_world2bcam = -1*R_world2bcam * location
    #
    # Use matrix_world instead to account for all constraints
    location, rotation = cam.matrix_world.decompose()[0:2]
    R_world2bcam = rotation.to_matrix().transposed()

    # Convert camera location to translation vector used in coordinate changes
    # T_world2bcam = -1*R_world2bcam*cam.location
    # Use location from matrix_world to account for constraints:     
    T_world2bcam = -1*R_world2bcam @ location

    # Build the coordinate transform matrix from world to computer vision camera
    R_world2cv = R_bcam2cv@R_world2bcam
    T_world2cv = R_bcam2cv@T_world2bcam

    # put into 3x4 matrix
    RT = Matrix((
        R_world2cv[0][:] + (T_world2cv[0],),
        R_world2cv[1][:] + (T_world2cv[1],),
        R_world2cv[2][:] + (T_world2cv[2],)
        ))
    return RT

def get_3x4_P_matrix_from_blender(cam):
    K = get_calibration_matrix_K_from_blender(cam.data)
    RT = get_3x4_RT_matrix_from_blender(cam)
    return K@RT, K, RT

################################################################################

import typing

class StereoImage(typing.NamedTuple):
    K: np.array
    pose: np.array
    baseline: float
    imL: np.array
    depthL: np.array
    imR: np.array
    depthR: np.array

def render():
    """ render active camera (image and depth) """
    
    bpy.context.scene.use_nodes = True
    tree = bpy.context.scene.node_tree
    links = tree.links

    for n in tree.nodes:
        tree.nodes.remove(n)

    rl = tree.nodes.new('CompositorNodeRLayers')

    v = tree.nodes.new('CompositorNodeViewer')
    v.use_alpha = True

    links.new(rl.outputs['Image'], v.inputs['Image'])
    # depth cannot be accessed in python; hack uses alpha to store z-values
    links.new(rl.outputs['Depth'], v.inputs['Alpha'])

    bpy.ops.render.render()
    pixels = bpy.data.images['Viewer Node']
    im = np.array(pixels.pixels[:]).reshape((pixels.size[1], pixels.size[0], pixels.channels))
    
    # set invalid depth values to 0.0
    im[:,:,3][im[:,:,3] >= _d_max] = 0.0
    return (im[:,:,0:2] * 255.0).astype(np.uint8), im[:,:,3]

def render_stereo(camera, baseline=0.15):
    bpy.context.scene.camera = camera
    _, K, pose = get_3x4_P_matrix_from_blender(camera)
    imL, depthL = render()
    
    location_old = camera.location.copy()
    camera.location = (camera.matrix_world @ Vector((baseline, 0.0, 0.0, 1.0)))[0:3]
    
    imR, depthR = render()
    
    camera_location = location_old
    
    return StereoImage(np.array(K), pose, baseline, imL, depthL, imR, depthR)


from ctypes import *
ftl = CDLL('/home/nick/git-repos/ftl/build/SDK/C/libftl-dev.so.0')

ftlCreateWriteStream = ftl.ftlCreateWriteStream
ftlCreateWriteStream.restype = c_void_p
ftlCreateWriteStream.argtypes = [c_char_p]

ftlIntrinsicsWriteLeft = ftl.ftlIntrinsicsWriteLeft
ftlIntrinsicsWriteLeft.restype = c_int
ftlIntrinsicsWriteLeft.argtypes = [c_void_p, c_int, c_int, c_int, c_float, c_float, c_float, c_float, c_float, c_float]

ftlImageWrite = ftl.ftlImageWrite
ftlImageWrite.restype = c_int
ftlImageWrite.argtypes = [c_void_p, c_int, c_int, c_int, c_int, c_void_p]

ftlDestroyStream = ftl.ftlDestroyStream
ftlDestroyStream.restype = c_int
ftlDestroyStream.argtypes = [c_void_p]

stream = ftlCreateWriteStream(b'./blender.ftl')

image = render_stereo(bpy.context.scene.camera, 0.15)
scale = bpy.context.scene.render.resolution_percentage / 100
resolution_x_in_px = scale * bpy.context.scene.render.resolution_x
resolution_y_in_px = scale * bpy.context.scene.render.resolution_y

err = ftlIntrinsicsWriteLeft(c_void_p(stream), c_int(0), c_int(int(resolution_x_in_px)), c_int(int(resolution_y_in_px)), c_float(300.0), c_float(-resolution_x_in_px/2), c_float(-resolution_y_in_px/2), c_float(0.1), c_float(0.1), c_float(8.0))
print(err)
err = ftlImageWrite(stream, 0, 0, 3, 0, image.imL.ctypes.data_as(c_void_p))
print(err)
err = ftlDestroyStream(stream)
print(err)

