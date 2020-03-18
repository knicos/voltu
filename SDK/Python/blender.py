bl_info = {
    "name": "FTL plugin",
    "blender": (2, 80, 2),
    "category": "Import-Export",
}

import bpy
import numpy as np
from mathutils import Matrix, Vector

from ftl import Camera, FTLStreamWriter
from ftl.types import Channel, Pipeline

################################################################################
# https://blender.stackexchange.com/a/120063
# https://blender.stackexchange.com/questions/15102/what-is-blenders-camera-projection-matrix-model

def get_sensor_size(sensor_fit, sensor_x, sensor_y):
    if sensor_fit == 'VERTICAL':
        return sensor_y
    return sensor_x

def get_sensor_fit(sensor_fit, size_x, size_y):
    if sensor_fit == 'AUTO':
        if size_x >= size_y:
            return 'HORIZONTAL'
        else:
            return 'VERTICAL'
    return sensor_fit


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

def get_ftl_calibration_from_blender(camd, d_min=0.0, d_max=np.inf, baseline=0.0, doff=0.0):

    K = get_calibration_matrix_K_from_blender(camd)
    scene = bpy.context.scene
    scale = scene.render.resolution_percentage / 100
    resolution_x_in_px = scale * scene.render.resolution_x
    resolution_y_in_px = scale * scene.render.resolution_y

    ftlcam = Camera(K[0][0], K[1][1], -K[0][2], -K[1][2],
                    int(resolution_x_in_px), int(resolution_y_in_px),
                    d_min, d_max, baseline, doff)

    return ftlcam

################################################################################

import typing

class StereoImage(typing.NamedTuple):
    intrinsics: Camera
    pose: np.array
    imL: np.array
    depthL: np.array
    imR: np.array
    depthR: np.array

def inverse_pose(pose):
    inv = np.identity(4, dtype=pose.dtype)
    inv[0:3,0:3] = pose[0:3,0:3].T
    inv[0,3] = -pose[0:3,3]@pose[0:3,0]
    inv[1,3] = -pose[0:3,3]@pose[0:3,1]
    inv[2,3] = -pose[0:3,3]@pose[0:3,2]
    return inv

def cycles_depth(depth, camd):
    xs = np.zeros(depth.shape, dtype=np.float)
    xs[:,:] = (0.5 - np.arange(0, xs.shape[1], dtype=np.float) / depth.shape[1]) * camd.sensor_width / camd.lens

    ys = np.zeros(depth.shape, dtype=np.float)
    (ys.T)[:,:] = (0.5 - np.arange(0, ys.shape[0], dtype=np.float) / depth.shape[0]) * camd.sensor_height / camd.lens

    norm = np.sqrt(xs*xs + ys*ys + 1.0)
    return depth / norm

def render():
    """ render active camera (image and depth) """
    context = bpy.context
    use_nodes = bool(context.scene.use_nodes)
    context.scene.use_nodes = True
    tree = context.scene.node_tree
    links = tree.links

    # possible issues with existing nodes of same type when accessing via bpy.data (?)
    rl = tree.nodes.new('CompositorNodeRLayers')
    v = tree.nodes.new('CompositorNodeViewer')
    v.use_alpha = True

    try:
        links.new(rl.outputs['Image'], v.inputs['Image'])
        # depth cannot be accessed in python; hack uses alpha to store z-values
        links.new(rl.outputs['Depth'], v.inputs['Alpha'])

        bpy.ops.render.render()

        pixels = bpy.data.images['Viewer Node']
        pix = np.array(pixels.pixels[:])

        # sRGB conversion
        pix_srgb = np.copy(pix)
        np.copyto(pix_srgb, 1.055*(pix**(1.0/2.4)) - 0.055, where=pix <= 1)
        np.copyto(pix_srgb, pix * 12.92, where=pix <= 0.0031308)

        # Clamp?
        pix_srgb[pix_srgb > 1.0] = 1.0

        im = pix_srgb.reshape((pixels.size[1], pixels.size[0], pixels.channels))[:,:,0:3]

        depth = np.array(pixels.pixels[:]).reshape((pixels.size[1], pixels.size[0], pixels.channels))[:,:,3]
        d_max = 65504.0
        depth[depth >= d_max] = 0.0 # set invalid depth values to 0.0

        if context.scene.render.engine == 'CYCLES':
             depth = cycles_depth(depth, context.scene.camera.data)

        return im, depth

    finally:
        tree.nodes.remove(v)
        tree.nodes.remove(rl)
        bpy.context.scene.use_nodes = use_nodes

def render_stereo(camera, baseline=0.15):
    context = bpy.context
    camera_old = bpy.context.scene.camera

    try:
        context.scene.camera = camera
        imL, depthL = render()

        matrix_world_old = camera.matrix_world.copy()
        try:
            camera.location = (camera.matrix_world @ Vector((baseline, 0.0, 0.0, 1.0)))[0:3]
            imR, depthR = render()

        finally:
            camera.matrix_world = matrix_world_old

    finally:
        context.scene.camera = camera_old

    pose = np.array(camera.matrix_world)
    pose[:,1] = -pose[:,1]
    pose[:,2] = -pose[:,2]

    d_max = max(np.max(depthL), np.max(depthR))
    ftlcamera = get_ftl_calibration_from_blender(camera.data, baseline=baseline, d_max=d_max)

    return StereoImage(ftlcamera, pose, imL, depthL, imR, depthR)

################################################################################

class FTL_Options(bpy.types.PropertyGroup):

    file_path : bpy.props.StringProperty(name="File",
                                         description="Output file",
                                         default="./blender.ftl",
                                         maxlen=1024,
                                         subtype="FILE_PATH")

    baseline : bpy.props.FloatProperty(name="Baseline",
                                       description="Distance between cameras (x-direction relative to camera)",
                                       default=0.15)

    use_sgm : bpy.props.BoolProperty(name="Use SGM",
                                     description="Calculate disparity using SGM (FTL)",
                                     default=False)


    use_ground_truth : bpy.props.BoolProperty(name="Save as ground truth",
                                              description="Save depth in Ground Truth instead of Depth channel",
                                              default=True)

    mask_occlusions : bpy.props.BoolProperty(name="Mask occlusions",
                                             description="Right camera depth is used to mask occluded pixels",
                                             default=True)

    depth_eevee :     bpy.props.BoolProperty(name="Render depth with Eevee",
                                             description="Always render depth with Eevee.",
                                             default=False)

    cameras : bpy.props.EnumProperty(
        name = "Cameras",
        description = "Cameras for rendering",
        items = [
            ("ACTIVE", "Active camera", "Only use active camera"),
            ("SELECTED", "Selected cameras", "Use all selected cameras"),
            ("ALL", "All cameras", "Use all available cameras")
        ],
        default = "ACTIVE"
    )

import ftl
class FTL_OT_Operator(bpy.types.Operator):
    bl_idname = "scene.ftl_operator"
    bl_label = "FTL Operator"

    def execute(self, context):
        options = context.scene.ftl_options
        writer = FTLStreamWriter(options.file_path)

        if options.use_sgm:
            writer.enable_pipeline(Pipeline.DEPTH)

        cameras = []

        if options.cameras == 'ACTIVE':
            cameras.append(bpy.context.scene.camera)

        elif options.cameras == 'SELECTED':
            for obj in context.selected_objects:
                if obj.type != 'CAMERA':
                    continue
                cameras.append(obj)

        elif options.cameras == 'ALL':
            for obj in context.scene.objects:
                if obj.type != 'CAMERA':
                    continue
                cameras.append(obj)

        depth_channel = Channel.Depth if not options.use_ground_truth else Channel.GroundTruth
        baseline = options.baseline

        for i, camera in enumerate(cameras):
            res = render_stereo(camera, baseline)
            writer.write(i, Channel.Calibration, res.intrinsics)
            writer.write(i, Channel.Pose, res.pose)
            writer.write(i, Channel.Left, res.imL)
            writer.write(i, Channel.Right, res.imR)

            depthL = res.depthL
            depthR = res.depthR

            if options.depth_eevee and context.scene.render.engine != 'BLENDER_EEVEE':
                engine = context.scene.render.engine
                try:
                    context.scene.render.engine = 'BLENDER_EEVEE'
                    res_eevee = render_stereo(camera, baseline)
                    depthL = res_eevee.depthL
                    depthR = res_eevee.depthR

                finally:
                    context.scene.render.engine = engine

            writer.write(i, depth_channel, depthL)

            if options.mask_occlusions:
                writer.mask_occlusion(i, depth_channel, depthR)

        print("saved to %s" % options.file_path)
        return {'FINISHED'}

class FTL_PT_Panel(bpy.types.Panel):
    bl_label = "FTL Export"
    bl_idname = "FTL_PT_ftl"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "output"

    def draw(self, context):
        layout = self.layout

        ftl_options = context.scene.ftl_options

        row = layout.row()
        row.prop(ftl_options, "file_path")

        row = layout.row()
        row.prop(ftl_options, "baseline")

        row = layout.row()
        row.prop(ftl_options, "cameras")

        row = layout.row()
        row.prop(ftl_options, "use_sgm")

        row = layout.row()
        row.prop(ftl_options, "use_ground_truth")

        row = layout.row()
        row.prop(ftl_options, "mask_occlusions")

        row = layout.row()
        row.prop(ftl_options, "depth_eevee")

        row = layout.row()
        row.operator("scene.ftl_operator", text="Generate")

classes = (FTL_Options,
           FTL_OT_Operator,
           FTL_PT_Panel)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.ftl_options = bpy.props.PointerProperty(type=FTL_Options)

def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.ftl_options

if __name__ == "__main__":
    register()
