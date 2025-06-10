This repository contains two key applications for photogrammetry workflows:

    1. Multi-Camera Turntable Capture (Python)

    2. CloudAlign: 3D Point Cloud Alignment & Analysis (C++/Eigen/PCL)

1. Multi-Camera Turntable Capture

Language: Python
Purpose: Automates synchronized image capture from multiple cameras mounted around a turntable, suitable for photogrammetry.

Key Features:

    Detects and prepares all connected cameras (tested with Nikon/Canon via gphoto2).

    Launches live video preview for each camera using ffmpeg and VLC.

    Controls a motorized turntable via USB serial (CH341 driver).

    Guides the user through setup and folder selection for image storage.

    Rotates the turntable to evenly spaced angles, capturing images from all cameras at each position.

    Organizes captured images by camera and angle for downstream 3D reconstruction.

Typical Use Case:
Quickly acquire a full set of images for photogrammetry by automating camera triggering and turntable rotation, minimizing manual intervention.

Multi-Camera Capture (Python):

    Requirements: Python 3, gphoto2, ffmpeg, VLC, tkinter, serial, and a compatible USB turntable.

    Run the script and follow the prompts to set up cameras, preview, and capture images.


2. CloudAlign: 3D Point Cloud Alignment & Analysis

Language: C++ (Eigen, PCL, happly, nlohmann/json)
Purpose: Aligns, compares, and analyzes two 3D point clouds (PLY files), providing quantitative similarity metrics and visualizations.


Key Features:

    Loads two PLY point clouds, auto-detects units (meters or millimeters), and normalizes scale.

    Aligns clouds using Principal Component Analysis (PCA) and Iterative Closest Point (ICP).

    Visualizes clouds before and after alignment.

    Computes similarity metrics: Hausdorff distance, Chamfer distance, RMSE, and Jaccard index.

    Generates colored heatmaps of geometric deviation (continuous and threshold-based).

    Saves heatmaps as PLY files and metrics as JSON for further analysis or reporting.

Typical Use Case:
Evaluate the geometric similarity between two 3D scans, validate reconstruction accuracy, or compare scans from different devices or methods.
Quick Start


    Requirements: C++17, Eigen, PCL, happly, nlohmann/json, nativefiledialog.

    Build and run the application. Select two PLY files when prompted.

    View visualizations, save heatmaps, and export similarity metrics.
