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
        source = o3d.read_point_cloud(SOURCE_PATH)
        target = o3d.read_point_cloud(TARGET_PATH)
        transform_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        source.transform(transform_init)
        return source, target

    def preprocess_point_cloud(self, pcd, voxel_size=0.01):
        "preprocess point cloud"
        print(":: Downsample with a voxel size %.3f." % voxel_size)
        pcd_down = o3d.voxel_down_sample(pcd, voxel_size)

        radius_normal = voxel_size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        o3d.estimate_normals(pcd_down,
                            o3d.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.compute_fpfh_feature(pcd_down,
                                            o3d.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
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
        self.ransac_result = o3d.registration_ransac_based_on_feature_matching(
                            source_down, target_down, source_fpfh, target_fpfh, DISTANCE_THRES,
                            o3d.TransformationEstimationPointToPoint(False), 4, 
                            [o3d.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                            o3d.CorrespondenceCheckerBasedOnDistance(DISTANCE_THRES)],
                            o3d.RANSACConvergenceCriteria(4000000, 500))
        dst_source.transform(self.ransac_result.transformation)
        return dst_source
    
    def execute_refine_registration(self, source, target, voxel_size=0.01):
        "execute ICP global registration"
        dst_source = copy.deepcopy(source)
        DISTANCE_THRES = voxel_size * 0.5
        print(":: Point-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % DISTANCE_THRES)
        self.icp_result = o3d.registration_icp(source, target, DISTANCE_THRES, self.ransac_result.transformation,
                                        o3d.TransformationEstimationPointToPlane())
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
        o3d.draw_geometries([source_temp, target_temp])
        return
    
    def save_coalition_3d(self, source, target, OUTPUT_PATH):
        "save point cloud"
        dst = source + target
        basename_without_ext = os.path.splitext(os.path.basename(OUTPUT_PATH))[0]
        o3d.write_point_cloud(basename_without_ext + "_dst.ply", dst)
        return

class Simple3DFilter():

    def __init__(self):
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
    SR.save_coalition_3d(icp_source, target, SOURCE_PATH)