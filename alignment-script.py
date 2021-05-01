import open3d as o3d
import numpy as np
import copy
import os
import sys

# parse command line arguments
try:
    source_folder = str(sys.argv[1])
    target_folder = str(sys.argv[2])
except:
    raise SystemExit(f"Missing first argument. \n Usage: <source folder> <target folder>")

# loads the corner version of the LIVOX sensor output
# estimates normal vectors, aligns them to the same plane, and filters ground points
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    # orient normals so the ground is facing the same way
    pcd_down.orient_normals_to_align_with_direction()
    pcd_down = remove_ground_points(pcd_down, 0.4)

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size, source_folder, target_folder):
    print(":: Load two point clouds")
    source = o3d.io.read_point_cloud(source_folder + "/corner.pcd")
    target = o3d.io.read_point_cloud(target_folder + "/corner.pcd")

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def remove_ground_points(pcd, z_threshold):
    pop_list = []
    for i, normal in enumerate(pcd.normals):
        if normal[2] > z_threshold:
            pop_list.append(i)

    print("Number of points to be removed: ", len(pop_list))
    pop_list.sort(reverse=True)
    for item in pop_list:
        pcd.normals.pop(item)
        pcd.points.pop(item)
    return pcd

# set voxel size for downsampling and ICP
voxel_size = 0.4
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
    voxel_size, source_folder, target_folder)

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
         3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(10000000, 0.999))
    return result

result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    radius_normal = voxel_size * 2
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    source.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    # orient normals so the ground is facing the same way
    target.orient_normals_to_align_with_direction()
    source.orient_normals_to_align_with_direction()
    # remove all points with a z normal component greater than the threshold
    target = remove_ground_points(target, 0.3)
    source = remove_ground_points(source, 0.3)
    result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                 voxel_size)

before_snow = o3d.io.read_point_cloud(source_folder + "/all_points.pcd")
after_snow = o3d.io.read_point_cloud(target_folder + "/all_points.pcd")

def process_all_points(pcd):
    pcd_down = pcd.voxel_down_sample(voxel_size=0.02)
    pcd_out, ind = pcd_down.remove_radius_outlier(nb_points=10, radius=0.06)
    return pcd_out

# process all points clouds
before_snow_out = process_all_points(before_snow)
after_snow_out = process_all_points(after_snow)
# apply calculated transformations and output the all points clouds for m3c2 distance calculation
before_snow_out = copy.deepcopy(before_snow_out).transform(result_icp.transformation)
o3d.io.write_point_cloud("before_aligned.pcd", before_snow_out)
o3d.io.write_point_cloud("after_aligned.pcd", after_snow_out)


# run cloudcompare in BASH command
# This assumes cloud compare was installed using the SNAPD package
# output is in cloud compare BIN file type
os.system('cloudcompare.CloudCompare -O "before_aligned.pcd" -O "after_aligned.pcd" -M3C2 "m3c2_params.txt" -FILTER_SF 0.06 100 -C_EXPORT_FMT "PCD"')

print("Calculation successful")
print("M3C2 Scalar field stored in local folder")
