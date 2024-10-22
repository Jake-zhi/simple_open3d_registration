#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""
simple_registration.py
"""

__author__  = "flow-dev"
__version__ = "1.00"
__date__    = "14 Mar 2020"

import copy
import os
import sys

import numpy as np
import open3d as o3d
print(o3d.__version__)


def get_args(arg_num):
    args = sys.argv
    if len(sys.argv) < arg_num:
        print("Missing arguments")
        sys.exit()
    else:
        return(args)

class SimpleRegistration():
    
    def __init__(self):
        return
    
    def read_point_clouds(self, SOURCE_PATH, TARGET_PATH):
        "read point clouds"
        source = o3d.io.read_point_cloud(SOURCE_PATH)
        target = o3d.io.read_point_cloud(TARGET_PATH)
        transform_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        source.transform(transform_init)
        return source, target

    def preprocess_point_cloud(self, pcd, voxel_size=0.01):
        "preprocess point cloud"
        print(":: Downsample with a voxel size %.3f." % voxel_size)
        pcd_down = pcd.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.registration.compute_fpfh_feature(pcd_down,
                                            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    def execute_global_registration(self, source, target, voxel_size=0.01):
        "execute RANSAC global registration"
        dst_source = copy.deepcopy(source)
        DISTANCE_THRES = voxel_size * 1.0
        source_down, source_fpfh = self.preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target, voxel_size)
        print(":: RANSAC registration on downsampled point clouds.")
        print("   Since the downsampling voxel size is %.3f," % voxel_size)
        print("   we use a liberal distance threshold %.3f." % DISTANCE_THRES)
        self.ransac_result = o3d.registration.registration_ransac_based_on_feature_matching(
                            source_down, target_down, source_fpfh, target_fpfh, DISTANCE_THRES,
                            o3d.registration.TransformationEstimationPointToPoint(False), 4, 
                            [o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                            o3d.registration.CorrespondenceCheckerBasedOnDistance(DISTANCE_THRES)],
                            o3d.registration.RANSACConvergenceCriteria(4000000, 500))
        dst_source.transform(self.ransac_result.transformation)
        return dst_source
    
    def execute_refine_registration(self, source, target, voxel_size=0.01):
        "execute ICP global registration"
        dst_source = copy.deepcopy(source)
        DISTANCE_THRES = voxel_size * 0.5
        print(":: Point-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % DISTANCE_THRES)
        self.icp_result = o3d.registration.registration_icp(source, target, DISTANCE_THRES, self.ransac_result.transformation,
                                        o3d.registration.TransformationEstimationPointToPlane())
        dst_source.transform(self.icp_result.transformation)
        return dst_source

    def draw_3d(self, source, target, FalseColor=False, transform=np.identity(4)):
        "draw point cloud"
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        if(FalseColor):
            source_temp.paint_uniform_color([1, 0.706, 0])
            target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transform)
        o3d.visualization.draw_geometries([source_temp, target_temp])
        return
    
    def save_coalition_3d(self, source, target, OUTPUT_PATH):
        "save point cloud"
        dst = source + target
        basename_without_ext = os.path.splitext(os.path.basename(OUTPUT_PATH))[0]
        o3d.io.write_point_cloud(basename_without_ext + "_coalition.ply", dst)
        return dst

class Simple3DFilter():

    def __init__(self):
        return

    def display_inlier_outlier(self, cloud, ind):
        inlier_cloud = cloud.select_down_sample(ind)
        outlier_cloud = cloud.select_down_sample(ind, invert=True)
        print("Showing outliers (red) and inliers (gray): ")
        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
        return

    def save_3d(self, source, OUTPUT_PATH):
        "save point cloud"
        basename_without_ext = os.path.splitext(os.path.basename(OUTPUT_PATH))[0]
        o3d.io.write_point_cloud(basename_without_ext + "_filterd.ply", source)
        return

    def execute_uniform_down_sample(self, cloud):
        print("Every 5th points are selected")
        uni_down_pcd = cloud.uniform_down_sample(every_k_points=5)
        o3d.visualization.draw_geometries([uni_down_pcd])
        return

    def execute_remove_statistical_outlier(self, cloud, voxel_size=0.001):
        print("Statistical oulier removal")
        voxel_down_pcd = cloud.voxel_down_sample(voxel_size)
        cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
        self.display_inlier_outlier(voxel_down_pcd, ind)
        dst = voxel_down_pcd.select_down_sample(ind)
        o3d.visualization.draw_geometries([dst])
        return dst

    def execute_remove_radius_outlier(self, cloud, voxel_size=0.001):
        print("Radius oulier removal")
        voxel_down_pcd = cloud.voxel_down_sample(voxel_size)
        cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.05)
        self.display_inlier_outlier(voxel_down_pcd, ind)
        dst = voxel_down_pcd.select_down_sample(ind)
        o3d.visualization.draw_geometries([dst])
        return dst

class SurfaceReconstruction():

    def __init__(self, pcd, OUTPUT_PATH):
        print('run Poisson surface reconstruction')
        self.mesh, self.densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
        o3d.visualization.draw_geometries([self.mesh])
        basename_without_ext = os.path.splitext(os.path.basename(OUTPUT_PATH))[0]
        o3d.io.write_triangle_mesh(basename_without_ext + "_surface_rec.ply", self.mesh)
        return
    
    def visualize_densities(self):
        print('visualize densities')
        self.densities = np.asarray(self.densities)
        import matplotlib.pyplot as plt
        density_colors = plt.get_cmap('plasma')(
                (self.densities - self.densities.min()) / (self.densities.max() - self.densities.min()))
        density_colors = density_colors[:, :3]
        density_mesh = o3d.geometry.TriangleMesh()
        density_mesh.vertices = self.mesh.vertices
        density_mesh.triangles = self.mesh.triangles
        density_mesh.triangle_normals = self.mesh.triangle_normals
        density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
        o3d.visualization.draw_geometries([density_mesh])
        return
    
    def remove_low_density_vertices(self):
        print('remove low density vertices')
        REMOVE_THRS = 0.1
        vertices_to_remove = self.densities < np.quantile(self.densities, REMOVE_THRS)
        self.mesh.remove_vertices_by_mask(vertices_to_remove)
        print(self.mesh)
        o3d.visualization.draw_geometries([self.mesh])
        return

if __name__ == "__main__":

    # get file path from args
    args = get_args(2)
    SOURCE_PATH = args[1]
    TARGET_PATH = args[2]
    print (SOURCE_PATH, TARGET_PATH)

    SR = SimpleRegistration()

    # read point clouds
    source, target = SR.read_point_clouds(SOURCE_PATH, TARGET_PATH)
    # execute global registration
    regist_source = SR.execute_global_registration(source, target)
    # execute refine registration
    icp_source = SR.execute_refine_registration(source, target)

    # draw
    SR.draw_3d(source, target)
#    SR.draw_3d(regist_source, target)
    SR.draw_3d(icp_source, target)

    # save
    merged_cloud = SR.save_coalition_3d(icp_source, target, TARGET_PATH)

    # remove outlier
    S3DF = Simple3DFilter()
#    S3DF.execute_uniform_down_sample(icp_source)
    statistical_cloud = S3DF.execute_remove_statistical_outlier(merged_cloud)
#    radius_cloud = S3DF.execute_remove_radius_outlier(merged_cloud)
    S3DF.save_3d(statistical_cloud, SOURCE_PATH)

    # create_from_point_cloud_poisson
    SFRC = SurfaceReconstruction(statistical_cloud, TARGET_PATH)
    SFRC.visualize_densities()
#    SFRC.remove_low_density_vertices()

