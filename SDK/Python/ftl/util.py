import numpy as np
from . types import Camera

def disparity_to_depth(disparity, camera, invalid_value=0.0):
    """ Calculate depth map from disparity map. Depth values smaller than 0.0
	    and larger than max_depth are set to invalid_value.
    """

    depth = (camera.fx * camera.baseline) / (disparity - camera.doff)
    depth[depth < 0] = invalid_value
    depth[depth > camera.max_depth] = invalid_value
    return depth

def depth_to_disparity(depth, camera, invalid_value=0.0):
    """ Calculate disparity from depth image. Inverse of disparity_to_depth().
    """
    valid = depth != invalid_value
    disparity = np.divide((camera.fx * camera.baseline), depth, where=valid) + camera.doff
    return disparity

def point3d(calib, u, v, d):
    """ Calculate point 3D coordinates
        @param  calib   calibration
        @param  u       image x coordinate
        @param  v       image y coordinate
        @param  d       depth value for pixel (u, v)
    """

    return np.array([(u+calib.cx)*d/calib.fx, (v+calib.cy)*d / calib.fy, d], dtype=np.float)

def depth_image_to_3d(depth, calib):
    """ Calculate 3D points from depth image and calibration paramters
        @param      depth   depth image
        @param      calib   calibration paramters
        @returns    3D points in (h,w,3) array
    """

    bad = (depth == 0.0) | (depth < calib.min_depth) | (depth > calib.max_depth)

    xs = np.zeros(depth.shape, dtype=np.float)
    xs[:,:] = (np.arange(0, xs.shape[1], dtype=np.float) + calib.cx) / calib.fx
    xs = xs * depth
    xs[bad] = 0.0

    ys = np.zeros(depth.shape, dtype=np.float)
    (ys.T)[:,:] = (np.arange(0, ys.shape[0], dtype=np.float) + calib.cy) / calib.fy
    ys = ys * depth
    ys[bad] = 0.0

    points = np.zeros((*depth.shape, 3), dtype=np.float)
    points[:,:,0] = xs
    points[:,:,1] = ys
    points[:,:,2] = np.where(bad, 0.0, depth)

    return points

def center_points_nonzero(points):
    if points.shape[-1] != 3:
        raise ValueError("last axis dimension must be 3")

    npoints = np.product(points.shape[:-1])
    rows = points.reshape((npoints, 3))

    rows[rows.nonzero()[0],:] -= [rows[rows.nonzero()[0],i].mean() for i in range(0, 3)]
    return points

def write_xyz(fname, points, color=None):
    """ Write XYZ file (for MeshLab etc).

        Points at origin (0, 0, 0) are not included.

        @param  fname   output file name
        @param  points  points in (n,3) or (h,w,3) array, where last dimension
                    contains 3D points in X, Y, Z order
        @param  color   RGB color image (optional)
    """

    if points.shape[-1] != 3:
        raise ValueError("last axis dimension must be 3")

    if len(points.shape) == 3:
        npoints = points.shape[0] * points.shape[1]
        rows = points.reshape((npoints, 3))

    elif len(points.shape) == 2:
        rows = points

    else:
        raise ValueError("points must be in (n,3) or (h,w,3) array")

    nonzero = rows.nonzero()[0]

    if color is not None:
        if color.shape[-1] != 3:
            raise ValueError("color must be rgb")

        with_color = np.zeros((rows.shape[0], 6), dtype=np.float)
        with_color[:,0:3] = rows
        with_color[:,3:6] = color.reshape((color.shape[0] * color.shape[1], 3))
        rows = with_color

    rows_nonzero = rows[nonzero,:]

    np.savetxt(fname, rows_nonzero, fmt=("%.9f "*rows_nonzero.shape[1]))

def write_ply(fname, points, color=None):
    """ Save points in PLY file as vertices.

        Points at origin (0, 0, 0) are not included.

        @param  fname   file name
        @param  points  points, last dimension 3 for point coordinates (x, y, z)
        @param  color   RGB color for points (optional)
    """
    if points.shape[-1] != 3:
        raise ValueError("last axis dimension for points must be 3 (x, y, z)")

    if color is not None:
        if color.shape[-1] != 3:
            raise ValueError("last axis for color must be 3 (r, g, b)")

        if np.product(points.shape[:-1]) != np.product(color.shape[:-1]):
            raise ValueError("color must have same dimensions as points")

    npoints_all = np.product(points.shape[:-1])
    points_ = points.reshape((npoints_all, 3))

    nonzero = points_.nonzero()[0]
    points_ = points_[nonzero,:]

    if color is not None:
        color_ = color.reshape((npoints_all, 3))[nonzero,:]

    npoints = points_.shape[0]

    with open(fname, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("element vertex %i\n" % npoints)
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")

        if color is not None:
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")

        f.write("end_header\n")

        if color is not None:
            for p, c in zip(points_, color_):
                f.write("%f %f %f %i %i %i\n" % (*p, *(c * 255)))

        else:
            for p in points_:
                f.write("%f %f %f\n" % p)
