#!/usr/bin/env python3.8
from collections import namedtuple
import sys
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

ply_point_cloud = o3d.data.PLYPointCloud()
# , format="xyzrgb"

point_cloud = sys.argv[1]
hull = int(sys.argv[2])
if point_cloud == "cloud_merged.pcd":
    pcd = o3d.io.read_point_cloud(point_cloud)
else:
    print(point_cloud)
    pcd = o3d.io.read_point_cloud(point_cloud, format="xyzrgb")

zoom = 0.5

RobotConfig = namedtuple("RobotConfig", ["ground_clearance", "robot_dimensions"])


def visualize(list_of_pcs: list):
    o3d.visualization.draw_geometries(
        list_of_pcs,
        zoom=zoom,
        front=[0.4257, -0.2125, -0.8795],
        lookat=[2.6172, 2.0475, 1.532],
        up=[-0.0694, -0.9768, 0.2024],
        # point_show_normal=True,
    )


# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
def dbscan_pcd(pcd):
    # labels = np.array(pcd.cluster_dbscan(eps=0.08, min_points=10, print_progress=True))
    labels = np.array(pcd.cluster_dbscan(eps=0.23, min_points=30, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0

    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    print(pcd)
    print(np.asarray(pcd.points))
    return (pcd, labels)


def plane_seg(pcd) -> tuple:
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.28, ransac_n=5, num_iterations=1000
    )
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers)
    return (inlier_cloud, inliers)


print("size befor down_sample: ", pcd)
down_pcd = pcd.voxel_down_sample(voxel_size=0.10)
print("size after: ", down_pcd)
print("get statistical outlier")
cl, outlier_idx = down_pcd.remove_statistical_outlier(nb_neighbors=500, std_ratio=0.5)
down_cloud = down_pcd.select_by_index(outlier_idx)
print("segmentation of in planes")
obstacles, obstacles_idx = plane_seg(down_cloud)
outlier_cloud = down_pcd.select_by_index(obstacles_idx + outlier_idx, invert=True)


def alpha_shape(pcd, alpha):
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    return mesh


def group_and_box_pcd(obstacles, hull: bool, alpha: float):
    """groups same points together and creates a bounding box and optionally a hull curve around the clusters"""
    obstacle_groups, labels_idx = dbscan_pcd(obstacles)
    print("Labels: ", labels_idx)

    max_label = labels_idx.max()

    clusters = [[] for _ in range(0, max_label + 1)]

    for idx, label in enumerate(labels_idx):
        if label >= 0:
            clusters[label].append(idx)

    visual = [obstacle_groups]
    for cluster in clusters:
        point_group = obstacle_groups.select_by_index(cluster)
        if hull:
            hull, _ = point_group.compute_convex_hull()
            hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
            hull_ls.paint_uniform_color((1, 0, 0))
            visual.append(hull_ls)
        # bb = point_group.get_axis_aligned_bounding_box()
        bb = point_group.get_oriented_bounding_box()
        bb.color = (1, 0, 0)
        bb.scale(1.25, bb.get_center())
        visual.append(bb)

    return visual


print("Recompute the normal of the downsampled point cloud")

# outlier_cloud.estimate_normals(
#     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
# )
# outlier_cloud.normalize_normals()

# normals = np.asarray(outlier_cloud.normals)


visual = group_and_box_pcd(obstacles, bool(hull), 0.03)
# visual.append(pcd)
visual.append(down_pcd)
visualize(visual)


