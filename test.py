from collections import namedtuple
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud("cloud_merged.pcd")
zoom = 0.5

RobotConfig = namedtuple("RobotConfig", ["ground_clearance", "robot_dimensions"])


def select_points_above(margin: int):
    """selects the points above a certain value above the av"""


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    # inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries(
        [inlier_cloud, outlier_cloud],
        zoom=0.3412,
        front=[0.4257, -0.2125, -0.8795],
        lookat=[2.6172, 2.0475, 1.532],
        up=[-0.0694, -0.9768, 0.2024],
    )


# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
def dbscan_pcd(pcd):
    # labels = np.array(pcd.cluster_dbscan(eps=0.08, min_points=10, print_progress=True))
    labels = np.array(pcd.cluster_dbscan(eps=0.2, min_points=10, print_progress=True))

    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0

    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries(
        [pcd],
        zoom=zoom,
        front=[0.4257, -0.2125, -0.8795],
        lookat=[2.6172, 2.0475, 1.532],
        up=[-0.0694, -0.9768, 0.2024],
    )


def plane_seg(pcd) -> tuple:
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.28, ransac_n=3, num_iterations=1000
    )
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return (inlier_cloud, outlier_cloud)


print("size befor down_sample: ", pcd)
down_pcd = pcd.voxel_down_sample(voxel_size=0.75)
print("size after: ", down_pcd)
cl, idx = down_pcd.remove_statistical_outlier(nb_neighbors=500, std_ratio=0.8)
inlier_cloud = down_pcd.select_by_index(idx)
inlier_cloud , _ = plane_seg(inlier_cloud)

o3d.visualization.draw_geometries(
    [
        inlier_cloud,
        p
    ],
    zoom=zoom,
    front=[-0.4999, -0.1659, -0.8499],
    lookat=[2.1813, 2.0619, 2.0999],
    up=[0.1204, -0.9852, 0.1215],
)

display_inlier_outlier(down_pcd, idx)


# aabb = pcd.get_axis_aligned_bounding_box()
# aabb.color = (1, 0, 0)
# obb = pcd.get_oriented_bounding_box()
# obb.color = (0, 1, 0)
# o3d.visualization.draw_geometries(
#         [down_pcd],
#         zoom=zoom,
#         front=[-0.4999, -0.1659, -0.8499],
#         lookat=[2.1813, 2.0619, 2.0999],
#         up=[0.1204, -0.9852, 0.1215],
#     )
