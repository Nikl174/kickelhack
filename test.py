#!/usr/bin/env python3.8
from collections import namedtuple
import sys
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

zoom = 0.5

RobotConfig = namedtuple("RobotConfig", ["ground_clearance", "robot_dimensions"])

# def travers_analysis(point_cloud, robot_config: RobotConfig) -> tuple:
def travers_analysis(point_cloud, hull: bool) -> tuple:
    """ find statistical outlier, create a plain around them and then cluster those plains to find trees and buildings"""
    print("Pointcloud: ", point_cloud)
    print("Size befor down_sample: ", point_cloud)
    down_pcd = point_cloud.voxel_down_sample(voxel_size=0.10)
    print("Size after: ", down_pcd)

    print("get statistical outlier")
    cl, outlier_idx = down_pcd.remove_statistical_outlier(nb_neighbors=500, std_ratio=0.5)
    down_cloud = down_pcd.select_by_index(outlier_idx)

    print("segmentation of in planes")
    obstacles, obstacles_idx = plane_seg(down_cloud)
    outlier_cloud = down_pcd.select_by_index(obstacles_idx , invert=True)
    visual, down_pcd = group_and_box_pcd(obstacles, down_pcd, bool(hull), 0.03)
    return visual, down_pcd
    
    

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
def dbscan_pcd(pcd, eps=0.23, min_points=30):
    """calls the DBSCAN algorithem to cluster the Point-cloud"""
    labels = np.array(pcd.cluster_dbscan(eps, min_points, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0

    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    print(pcd)
    print(np.asarray(pcd.points))
    return (pcd, labels)


def plane_seg(pcd, distance_threshold=0.20, ransac_n=5, num_iterations=1000) -> tuple:
    plane_model, inliers = pcd.segment_plane(
        distance_threshold, ransac_n, num_iterations
    )
    """find a plain in the point_cloud"""
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers)
    return (inlier_cloud, inliers)



def group_and_box_pcd(obstacles, pcl, hull: bool, alpha: float, ):
    """groups same points together and creates a bounding box and optionally a hull curve around the clusters"""
    obstacle_groups, labels_idx = dbscan_pcd(obstacles)
    print("Labels: ", labels_idx)
    pcd = pcl
    max_label = labels_idx.max()

    clusters = [[] for _ in range(0, max_label + 1)]

    # get the cluster from the returned labels
    for idx, label in enumerate(labels_idx):
        if label >= 0:
            clusters[label].append(idx)

    # generate a bounding box (and convex hull) for each cluster
    visual = [obstacle_groups]
    for cluster in clusters:
        point_group = obstacle_groups.select_by_index(cluster)
        if False:
            hull, _ = point_group.compute_convex_hull()
            hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
            hull_ls.paint_uniform_color((1, 0, 0))
            visual.append(hull_ls)
        # bb = point_group.get_axis_aligned_bounding_box()
        bb = point_group.get_oriented_bounding_box()
        bb.color = (1, 0, 0)
        bb.scale(1.50, bb.get_center())
        # bb.transform(np.array([[1,0,0,0],[0,1,0,0],[0,0,-2]]))
        points_in_bb = bb.get_point_indices_within_bounding_box(pcl.points)
        pcd = pcd.crop(bb, invert=True)
        visual.append(bb)

    return (visual, pcd)



def main():
    if len(sys.argv)<2:
        print("Usage: ", sys.argv[0], " path/to/pcd_file.pcd")
    point_cloud = sys.argv[1]
    if point_cloud != "HiL_Innen_subsampled.asc":
        pcd = o3d.io.read_point_cloud(point_cloud)
    else:
        print(point_cloud)
        pcd = o3d.io.read_point_cloud(point_cloud, format="xyzrgb")

    # if point_cloud=="down_pcd.pcd":
    #     orig_pcd = o3d.io.read_point_cloud("cloud_merged.pcd")
    #     plein_pcd, inliner = plane_seg(pcd)
    #     plain = pcd.select_by_index(inliner)
    #     plain.paint_uniform_color([1,0,0])
    #     visualize([plain, orig_pcd])
    # else:
    visual, down_pcd = travers_analysis(pcd, False)
    plein_pcd, inliner = plane_seg(pcd)
    plain = pcd.select_by_index(inliner)
    plain.paint_uniform_color([0,0,1])
    visual.append(plein_pcd)
    visual.append(plain)
    visual.append(down_pcd)
    visualize(visual)
    
main()
