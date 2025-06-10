import tkinter as tk
from tkinter import filedialog
import numpy as np
import open3d as o3d


#================================================================================================================================

def selectFile():
    """
    A method to select a file using fileDialog
    """
    root = tk.Tk()
    root.withdraw()
    source_path = filedialog.askopenfilename(title="Select source file")
    target_path = filedialog.askopenfilename(title="Select target file")
    
    if not (source_path and target_path):
        print("Missing files!")
        return
    
    return source_path, target_path

#================================================================================================================================

def processFiles(sourceFile, targetFile):
    """
    A method to load files and returns point clouds
    """
    source = o3d.io.read_point_cloud(sourceFile)
    target = o3d.io.read_point_cloud(targetFile)
    source_down = source.voxel_down_sample(voxel_size=0.8)
    target_down = target.voxel_down_sample(voxel_size=0.8)
    source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    return source_down, target_down

#================================================================================================================================

def visualize(source, target):
    """
    A method to visualize the source and target file on the same frame
    """
    source_pcd = o3d.geometry.PointCloud()
    target_pcd = o3d.geometry.PointCloud()

    source_points = np.asarray(source.points)
    target_points = np.asarray(target.points)

    source_pcd.points = o3d.utility.Vector3dVector(source_points)
    target_pcd.points = o3d.utility.Vector3dVector(target_points)

    source_pcd.paint_uniform_color([1, 0, 0])  # Red for source
    target_pcd.paint_uniform_color([0, 1, 0])  # Green for target

    print(f"Source point cloud: {len(source_pcd.points)} points")
    print(f"Target point cloud: {len(target_pcd.points)} points")

    print("Visualizing point clouds...")
    o3d.visualization.draw_geometries([source_pcd, target_pcd])

#================================================================================================================================

def FeatureBasedGlobalRegistration(source, target):
    # Compute FPFH features
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source, 
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target,
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100))

    # Fixed parameters with mutual_filter and updated criteria
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, 
        source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=0.05,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.05)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            max_iteration=4000000,
            confidence=0.999  
        )
    )
    return result.transformation

#================================================================================================================================

def pointToPlaneICP(source, target, threshold, initialTransform):

    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, threshold, initialTransform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return reg_p2l

    

#================================================================================================================================


def main():
    sourcePath, targetPath = selectFile()
    source, target = processFiles(sourcePath, targetPath)
    initialTransform = FeatureBasedGlobalRegistration(source, target)
    ICPTransform = pointToPlaneICP(source, target, 0.02, initialTransform)
    source.transform(ICPTransform)
    visualize( source, target)

#================================================================================================================================


if __name__ == "__main__":
    main()