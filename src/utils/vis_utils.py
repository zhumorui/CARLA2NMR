import numpy as np
import open3d as o3d
import cv2

def draw_camera(K, R, t, w, h, scale=1, color=[0.8, 0.2, 0.8], img_path=None):
    """Create axis, plane and pyramed geometries in o3d format.
    :param K: calibration matrix (camera intrinsics)
    :param R: rotation matrix
    :param t: translation
    :param w: image width
    :param h: image height
    :param scale: camera model scale
    :param color: color of the image plane and pyramid lines
    :param img_path: path to the image file
    :return: camera model geometries (axis, plane and pyramid)
    """

    # intrinsics
    K = K.copy() / scale
    Kinv = np.linalg.inv(K)

    # 4x4 transformation
    T = np.column_stack((R, t))
    T = np.vstack((T, (0, 0, 0, 1)))

    # axis
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.5 * scale
    )
    
    axis.transform(T)

    # points in pixel
    points_pixel = [
        [0, 0, 0],
        [0, 0, 1],
        [w, 0, 1],
        [0, h, 1],
        [w, h, 1],
    ]

    # pixel to camera coordinate system
    points = [Kinv @ p for p in points_pixel]


    
    img = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2RGB)

    # image plane
    width = abs(points[1][0]) + abs(points[3][0])
    height = abs(points[1][1]) + abs(points[3][1])

    w, h, = width, height

    # Normalize the vertex positions by the maximum dimension to maintain aspect ratio
    vertices = np.array([
        [0, 0, 0],   # Bottom-left
        [w, 0, 0],   # Bottom-right
        [w, h, 0],   # Top-right
        [0, h, 0]    # Top-left
    ])  # Normalize to keep aspect ratio

    faces = np.array([
        [0, 1, 2],  # Lower triangle
        [2, 3, 0],  # Upper triangle
    ])
    
    # Set UV coordinates
    v_uv = np.array([
        [0, 0],  # UV for Top-left
        [1, 0],  # UV for Top-right
        [1, 1],  # UV for Bottom-right
        [0, 1]   # UV for Bottom-left
    ])


    # Correct faces' UV indexing, each face needs 3 UV coordinates, one for each vertex
    face_uvs = np.array([
        [0, 1, 2],  # UVs for the first triangle
        [2, 3, 0]   # UVs for the second triangle
    ])

    # Create the mesh
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
    mesh.textures = [o3d.geometry.Image(img)]
    mesh.triangle_uvs = o3d.utility.Vector2dVector(v_uv[face_uvs.flatten()])
    mesh.triangle_material_ids = o3d.utility.IntVector([0] * len(faces))
        
    mesh.translate([points[1][0], points[1][1], scale])
    mesh.transform(T)


    # pyramid
    points_in_world = [(R @ p + t) for p in points]
    lines = [
        [0, 1],
        [0, 2],
        [0, 3],
        [0, 4],
    ]
    colors = [color for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points_in_world),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # return as list in o3d format
    return axis, mesh, line_set