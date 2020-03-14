#!/usr/bin/python
# -*- coding: utf-8 -*-

import open3d as o3d
import sys
import pprint

pprint.pprint(sys.path)
#print(o3d.__version__)

def display_inlier_outlier(cloud, ind):
    inlier_cloud = o3d.select_down_sample(cloud, ind)
    outlier_cloud = o3d.select_down_sample(cloud, ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.draw_geometries([inlier_cloud, outlier_cloud])

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.read_point_cloud("sample_gundum_01_dst.ply")
    o3d.draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.02")
    voxel_down_pcd = o3d.voxel_down_sample(pcd, voxel_size=0.01)
    o3d.draw_geometries([voxel_down_pcd])

    print("Every 5th points are selected")
    uni_down_pcd = o3d.uniform_down_sample(pcd, every_k_points=5)
    o3d.draw_geometries([uni_down_pcd])

    print("Statistical oulier removal")
    cl, ind = o3d.statistical_outlier_removal(voxel_down_pcd, nb_neighbors=20, std_ratio=2.0)
    display_inlier_outlier(voxel_down_pcd, ind)
    

    print("Radius oulier removal")
    cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.05)
    display_inlier_outlier(voxel_down_pcd, ind)