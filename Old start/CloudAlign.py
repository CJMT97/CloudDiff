import tkinter as tk
from tkinter import filedialog
import numpy as np
import open3d as o3d


def downsampleAndSave(file):
    """
    This method exists to create downsampled ply files for faster loading and processing.
    Support method only. 
    """

    data = o3d.io.read_point_cloud(file)
    dataPCD = data.farthest_point_down_sample(30000)
    filename = filedialog.asksaveasfilename(
        title="Save sampled PLY file as",
        defaultextension=".ply",
        filetypes=[("PLY files", "*.ply"), ("All files", "*.*")]
    )
    o3d.io.write_point_cloud(filename, dataPCD, write_ascii=True)
    print(f"Wrote data to {filename} successfully.")

#================================================================================================================================


def reshapeData(file):
    """
    This method is used to manipulate the source file by 
    1. translation along any axis
    2. Scale
    3. Rotate along any axis
    It writes the output to a ply file by asking the user 
    for a filename and destination folder
    """
    dataCloud = o3d.io.read_point_cloud(file)
    data = np.asarray(dataCloud.points)
    




#================================================================================================================================


def selectFile():
    """
    A method to select the source and target files using fileDialog
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
    A method that reads ply files and returns NdArrays ready for processing.
    Downsampling is done for fast processing. Furthest point down sample is used for better representation.
    Downsampling Will be removed during 'production'
    """
    source = o3d.io.read_point_cloud(sourceFile)
    target = o3d.io.read_point_cloud(targetFile)
    sourcePCD = source.farthest_point_down_sample(30000)
    targetPCD = target.farthest_point_down_sample(30000)
    sourceArray = np.asarray(sourcePCD.points)
    targetArray = np.asarray(targetPCD.points)
    sourceArray = np.transpose(sourceArray)
    targetArray = np.transpose(targetArray)

    return sourceArray, targetArray

#================================================================================================================================

def visualize(source, target):
    """
    A method that visualizes a source and target on the same frame.
    The source is red and the target is green.
    This method is for debugging and support.
    """
    # Create point clouds from the arrays
    source_pcd = o3d.geometry.PointCloud()
    target_pcd = o3d.geometry.PointCloud()
    
    # Convert data format - assumes input is (3, n)
    source_points = np.transpose(source)
    target_points = np.transpose(target)

    # Set points
    source_pcd.points = o3d.utility.Vector3dVector(source_points)
    target_pcd.points = o3d.utility.Vector3dVector(target_points)
    
    # Color the point clouds
    source_pcd.paint_uniform_color([1, 0, 0])  # Red for source
    target_pcd.paint_uniform_color([0, 1, 0])  # Green for target
    
    # Print information about the point clouds
    print(f"Source point cloud: {len(source_pcd.points)} points")
    print(f"Target point cloud: {len(target_pcd.points)} points")
    
    # Visualize
    print("Visualizing point clouds...")
    o3d.visualization.draw_geometries([source_pcd, target_pcd])

#================================================================================================================================


def centreAndScale(source, target):
    """
    This method centres both ply files around the origin and then matches the scale of the 
    source to the target.
    Returns centred and scaled NdArrays
    """
    centroid1 = np.mean(source, axis=1)
    centroid2 = np.mean(target, axis=1)
    sourceCentred = source - centroid1
    targetCentred = target - centroid2
    scale_factor = np.std(targetCentred) / np.std(sourceCentred)
    sourceScaled = sourceCentred * scale_factor
    return sourceScaled, targetCentred


#================================================================================================================================

def main():
    #Select files
    sourceFile, targetFile = selectFile()
    #Get files as NdArrays of size (3,N)
    source, target = processFiles(sourceFile, targetFile)
    #Visualize the source and target
    visualize(source,target)



    """
    Extra code for manipulations, support, debugging etc.
    """
    # downsampleAndSave(sourceFile)
    # downsampleAndSave(targetFile)



if __name__ == "__main__":
    main()