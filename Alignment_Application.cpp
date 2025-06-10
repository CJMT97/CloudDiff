#include "happly.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <array>
#include <algorithm>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <nfd.h>
#include <nlohmann/json.hpp>
#include <unordered_set>
#include <chrono>

// Constants
const double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062;

// Heatmap configuration structure
struct Heatmap {
	float min_value = -1.0f;
	float max_value = 1.0f;
	float zero_threshold = 0.0f;
};

// Used to store similarity metrics between our two point clouds
struct SimilarityMetrics {
	double hausdorff_distance;
	double chamfer_distance;
	double rmse;
	double jaccard_index;
};

// std::clamp is a C++ 17 exclusive, and it is easier to just define this
// rather than specify settings for the compiler. 
template<typename T>
T clamp(T value, T min_val, T max_val) {
	return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

/** Detects unit of a PLY file based on coordinate statistics */
enum class Unit { METERS, MILLIMETERS };

Unit detectUnit(const std::vector<std::array<double, 3>>& points) {
	// Compute the bounding box diagonal length
	double minVal = std::numeric_limits<double>::max();
	double maxVal = std::numeric_limits<double>::lowest();
	for (const auto& p : points) {
		for (int i = 0; i < 3; ++i) {
			minVal = std::min(minVal, p[i]);
			maxVal = std::max(maxVal, p[i]);
		}
	}
	double span = std::abs(maxVal - minVal);

	// Heuristic: If the span is >10, it's probably mm; if <10, probably m
	// Adjust threshold as needed for your datasets
	if (span > 100.0) return Unit::MILLIMETERS;
	return Unit::METERS;
}


/**
 * A method to get a file path using NFD (Native File Dialog).
 * nfd is cross-platform, which is very useful as many researchers use macs.
 * This method opens a file dialog to select a file with the specified filter.
 * @param filter File extension filter (default is "ply")
 * @return Selected file path as a string
*/
std::string getFilePathNFD(const char* filter = "ply") {

	nfdchar_t* outPath = nullptr;

	nfdu8filteritem_t filterItem = { "PLY Files", filter };
	nfdopendialogu8args_t args = { 0 };
	args.filterList = &filterItem;
	args.filterCount = 1;

	nfdresult_t result = NFD_OpenDialogU8_With(&outPath, &args);	
	
	std::string path;

	if (result == NFD_OKAY) {
		path = outPath;
		free(outPath);
	}
	else if (result == NFD_CANCEL) {
		std::cout << "User pressed cancel." << std::endl;
	}
	else {
		std::cerr << "Error: " << NFD_GetError() << std::endl;
	}
	return path;
}

/***
 * A method to get a save file path using NFD (Native File Dialog).
 * This method opens a save dialog to specify a file path for saving.
 * By default, it saves ply files, but this is also used to save our JSON object.
 * This method is just used because happly will happily (heh) overwrite existing files,
 * which could be inconvenient. 
 * @param defaultName Default file name for the save dialog
 * @return Selected file path as a string
*/
std::string getSaveFilePathNFD(const char* defaultName = "heatmap.ply") {
	nfdchar_t* outPath = nullptr;

	// Extract extension from default name
	std::string defaultStr(defaultName);
	size_t dotPos = defaultStr.find_last_of('.');
	std::string extension = (dotPos != std::string::npos) ? defaultStr.substr(dotPos + 1) : "ply";

	// Configure filter based on extension
	nfdu8filteritem_t filterItem[2];
	nfdsavedialogu8args_t args = { 0 };

	if (extension == "json") {
		filterItem[0] = { "JSON Files", "json" };
		args.filterList = filterItem;
		args.filterCount = 1;
	}
	else {
		// Default to PLY filter
		filterItem[0] = { "PLY Files", "ply" };
		args.filterList = filterItem;
		args.filterCount = 1;
	}

	args.defaultName = defaultName;

	nfdresult_t result = NFD_SaveDialogU8_With(&outPath, &args);

	std::string path;
	if (result == NFD_OKAY) {
		path = outPath;
		NFD_FreePathU8(outPath);

		// Auto-append extension if missing
		if (path.find('.') == std::string::npos) {
			path += "." + extension;
		}
	}
	return path;
}

/**
 * A method to create a transformation matrix from translation, rotation, and scale.
 * This method constructs a 4x4 transformation matrix that can be applied to point clouds.
 * @param translation Translation vector (X, Y, Z)
 * @param rotation_angles_deg Rotation angles in degrees (X, Y, Z)
 * @param scale Scale factors (X, Y, Z)
 * @return 4x4 transformation matrix
*/
Eigen::Matrix4d createTransform(const Eigen::Vector3d& translation,
	const Eigen::Vector3d& rotation_angles_deg,
	const Eigen::Vector3d& scale) {

	// Convert degrees to radians
	Eigen::Vector3d rotation_rad = rotation_angles_deg * (pi / 180.0);

	// Create affine transformation
	Eigen::Affine3d T = Eigen::Affine3d::Identity();

	// Apply transformations in order: Scale -> Rotate -> Translate
	T.translate(translation)
		.rotate(Eigen::AngleAxisd(rotation_rad.z(), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(rotation_rad.y(), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(rotation_rad.x(), Eigen::Vector3d::UnitX()))
		.scale(scale);

	return T.matrix();
}

/**
 * A method to apply a transformation matrix to a point cloud represented as an Eigen matrix.
 * This method applies the transformation matrix to each point in the cloud.
 * @param cloud Point cloud as Eigen matrix (3xN)
 * @param transform 4x4 transformation matrix
*/
void applyTransform(Eigen::MatrixXd& cloud, const Eigen::Matrix4d& transform) {
	// Convert to homogeneous coordinates (4xN)
	Eigen::MatrixXd homog_cloud(4, cloud.cols());
	homog_cloud.topRows(3) = cloud;
	homog_cloud.row(3).setOnes();

	// Apply transformation
	homog_cloud = transform * homog_cloud;

	// Convert back to 3xN and overwrite original
	cloud = homog_cloud.topRows(3);
}

/**
 * A method to apply a transformation to a point cloud represented as an Eigen matrix.
 * This method applies the transformation matrix to each point in the cloud.
 * It first calls the createTransform() to create a single transformation matrix from the 
 * translation, rotation, and scale parameters and then calls applyTransform(). This is efficient.
 * @param source Source point cloud as Eigen matrix
 * @param translation Translation vector (X, Y, Z)
 * @param rotation_deg Rotation angles in degrees (X, Y, Z)
 * @param scale Scale factors (X, Y, Z)
 * @return Transformed point cloud as Eigen matrix
*/
Eigen::MatrixXd applyTestTransform(
	const Eigen::MatrixXd& source,
	const Eigen::Vector3d& translation,
	const Eigen::Vector3d& rotation_deg,
	const Eigen::Vector3d& scale)
{
	// Create copy of source cloud
	Eigen::MatrixXd transformed = source;

	// Construct transformation matrix
	Eigen::Matrix4d transform = createTransform(
		translation,
		rotation_deg,
		scale
	);

	// Apply transformation to the copy
	applyTransform(transformed, transform);

	return transformed;
}


/**
 * Method to convert Eigen matrix to PCL's PointCloud. Used at multiple points in
 * main() for visualization and analysis. Sometimes PCL is needed over Eigen.
 * This method converts a 3xN Eigen matrix to a PCL PointCloud<pcl::PointXYZ>.
 * It iterates through the columns of the Eigen matrix, creating a PointXYZ for each column.
 * @param mat Input Eigen matrix (3xN)
 * @return Shared pointer to the resulting PCL PointCloud<pcl::PointXYZ>
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr eigenToPCL(const Eigen::MatrixXd& mat) {
	auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	for (int i = 0; i < mat.cols(); ++i) {  // Iterate columns
		pcl::PointXYZ pt;
		pt.x = mat(0, i);  // First row = X coordinates
		pt.y = mat(1, i);  // Second row = Y
		pt.z = mat(2, i);  // Third row = Z
		cloud->push_back(pt);
	}
	return cloud;
}

/**
 * A method to compute the alignment error between two point clouds.
 * This method uses a KdTree to find the nearest neighbor in the target cloud for each point in the source cloud.
 * The error is computed as the sum of distances from each source point to its nearest target point.
 * THis is useful in finding the correct set of axes during PCA, as axes can get flipped during alignment.
 * @param source Source point cloud as Eigen matrix
 * @param target Target point cloud as Eigen matrix
 * @return Total alignment error (sum of distances)
*/
double computeAlignmentError(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud = eigenToPCL(target);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(targetCloud);

	double totalError = 0.0;
	for (int i = 0; i < source.cols(); ++i) {
		Eigen::Vector3d pt = source.col(i);
		std::vector<int> indices(1);
		std::vector<float> distances(1);

		if (kdtree.nearestKSearch(pcl::PointXYZ(pt.x(), pt.y(), pt.z()), 1, indices, distances) > 0) {
			totalError += std::sqrt(distances[0]);
		}
	}
	return totalError;
}

/**
 * A method to optimize PCA axes by testing all combinations of axis inversions.
 * This method finds the best orientation of the PCA axes that minimizes the alignment error.
 * Otherwise PCA can result in aligned point clouds facing in opposite directions.
 * @param source Source point cloud as Eigen matrix
 * @param target Target point cloud as Eigen matrix
 * @param sortedEigenvectors1 Eigenvectors from source PCA
 * @param sortedEigenvectors2 Eigenvectors from target PCA
 */
void optimizePCAAxes(Eigen::MatrixXd& source,
	const Eigen::MatrixXd& target,
	Eigen::Matrix3d& sortedEigenvectors1,
	Eigen::Matrix3d& sortedEigenvectors2) {

	Eigen::Matrix3d bestBasis = sortedEigenvectors2;
	double minError = std::numeric_limits<double>::max();

	// Test all 8 possible inversion combinations
	for (int x_flip = 0; x_flip < 2; ++x_flip) {
		for (int y_flip = 0; y_flip < 2; ++y_flip) {
			for (int z_flip = 0; z_flip < 2; ++z_flip) {
				Eigen::Matrix3d testBasis = sortedEigenvectors2;
				if (x_flip) testBasis.col(0) *= -1;
				if (y_flip) testBasis.col(1) *= -1;
				if (z_flip) testBasis.col(2) *= -1;

				Eigen::Matrix3d R = testBasis * sortedEigenvectors1.transpose();
				Eigen::MatrixXd transformed = R * source;
				double currentError = computeAlignmentError(transformed, target);

				if (currentError < minError) {
					minError = currentError;
					bestBasis = testBasis;
				}
			}
		}
	}

	// Apply the correct combination of axes to the source
	sortedEigenvectors2 = bestBasis;
}


/**
 * A method to perform PCA alignment of two point clouds.
 * This method computes the PCA of both point clouds, aligns them, and returns the aligned source.
 * @param points1 First point cloud as Eigen matrix
 * @param points2 Second point cloud as Eigen matrix
 */
int PCA(Eigen::MatrixXd& points1, Eigen::MatrixXd& points2) {

	// Compute means (centroids)
	Eigen::Vector3d m1 = points1.rowwise().mean();
	Eigen::Vector3d m2 = points2.rowwise().mean();

	// Centre points
	points1.colwise() -= m1;
	points2.colwise() -= m2;

	// Get average length
	double len1 = (points1.colwise().norm()).mean();
	double len2 = (points2.colwise().norm()).mean();

	//std::cout << len1 << " vs " << len2 << std::endl;
	points1 /= len1;
	points2 /= len2;

	// Covariance Matrix
	Eigen::Matrix3d C1 = points1 * points1.transpose();

	//std::cout << C1 << std::endl;

	auto sortedEigenvectors = [](Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>& solver) {
		return solver.eigenvectors().rowwise().reverse();
		};

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig1(C1);
	Eigen::Matrix3d sortedEigenvectors1 = sortedEigenvectors(eig1);
	//std::cout << "Eigenvalues for Eig1:\n" << eig1.eigenvalues() << std::endl;
	//std::cout << "Eigenvectors for Eig1:\n" << eig1.eigenvectors() << std::endl;

	Eigen::Matrix3d C2 = points2 * points2.transpose();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig2(C2);
	Eigen::Matrix3d sortedEigenvectors2 = sortedEigenvectors(eig2);
	//std::cout << "Eigenvalues for Eig2:\n" << eig2.eigenvalues() << std::endl;
	//std::cout << "Eigenvectors for Eig2:\n" << eig2.eigenvectors() << std::endl;

	optimizePCAAxes(points1, points2, sortedEigenvectors1, sortedEigenvectors2);

	Eigen::Matrix3d estR = sortedEigenvectors2 * sortedEigenvectors1.transpose();

	if (estR.determinant() < 0) {
		estR.col(2) *= -1;  
		estR.row(2) *= -1;  
	}

	points1 = estR * points1;

	return 0;
}


/**
  * A method to visualize Point Clouds with a custom title
  * This is used for visual debugging, but is not necessary for the main functionality.
  * Calls to visualize() in main() can be commented out, and can improve speed somewhat.
  * @param mat1 First point cloud as Eigen matrix
  * @param mat2 Second point cloud as Eigen matrix
  * @param title Window title for visualization
  */
void visualize(const Eigen::MatrixXd& mat1,
	const Eigen::MatrixXd& mat2,
	const std::string& title = "Cloud Viewer") {
	auto cloud1 = eigenToPCL(mat1);
	auto cloud2 = eigenToPCL(mat2);

	pcl::visualization::PCLVisualizer::Ptr viewer(
		new pcl::visualization::PCLVisualizer(title));
	viewer->setBackgroundColor(0.05, 0.05, 0.05);
	viewer->addCoordinateSystem(0.1); // 10cm scale


	// Add first cloud (green)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud1, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud1, color1, "cloud1");
	viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");

	// Add second cloud (red)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud2, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud2, color2, "cloud2");
	viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");

	viewer->addCoordinateSystem(1.0);

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
}

/**
  * A method to perform Iterative Closest Point (ICP) alignment of two point clouds.
  * @param source_mat Source point cloud as Eigen matrix (3xN)
  * @param target_mat Target point cloud as Eigen matrix (3xM)
  * @param aligned_source Output matrix for aligned source point cloud (3xN)
  * @param max_iter Maximum number of ICP iterations (default is 50). Increase if the alignment is not good enough.
  * @param max_corr_dist Maximum correspondence distance for ICP (default is 0.1f). Decrease for more precise alignment.
  */
Eigen::Matrix4f performICP(const Eigen::MatrixXd& source_mat,
	const Eigen::MatrixXd& target_mat,
	Eigen::MatrixXd& aligned_source,
	int max_iter = 50,
	float max_corr_dist = 0.1f) {
	// Convert to PCL format
	auto source_cloud = eigenToPCL(source_mat);
	auto target_cloud = eigenToPCL(target_mat);

	// Create ICP instance
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(source_cloud);
	icp.setInputTarget(target_cloud);
	icp.setMaximumIterations(max_iter);
	icp.setMaxCorrespondenceDistance(max_corr_dist);

	// Perform alignment
	pcl::PointCloud<pcl::PointXYZ> final_cloud;
	icp.align(final_cloud);

	if (!icp.hasConverged()) {
		std::cerr << "ICP failed to converge!" << std::endl;
		return Eigen::Matrix4f::Identity();
	}

	// Get transformation matrix
	Eigen::Matrix4f transformation = icp.getFinalTransformation();

	// Convert aligned cloud back to Eigen
	aligned_source.resize(3, source_mat.cols());
	for (size_t i = 0; i < final_cloud.size(); ++i) {
		aligned_source(0, i) = final_cloud[i].x;
		aligned_source(1, i) = final_cloud[i].y;
		aligned_source(2, i) = final_cloud[i].z;
	}

	return transformation;
}

/**
 * Save a colored point cloud (heatmap) to a PLY file using happly.
 * This file can then be later opened in other programs like meshlab for furhter analysis.
 * @param cloud PCL PointCloud<PointXYZRGB> pointer (heatmap)
 * @param filename Output PLY file name 
 */
void saveHeatmapToPLY(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename) {
	std::vector<std::array<double, 3>> positions;
	std::vector<std::array<unsigned char, 3>> colors;

	positions.reserve(cloud->size());
	colors.reserve(cloud->size());

	for (const auto& pt : cloud->points) {
		positions.push_back({ static_cast<double>(pt.x), static_cast<double>(pt.y), static_cast<double>(pt.z) });
		colors.push_back({ pt.r, pt.g, pt.b });
	}

	happly::PLYData plyOut;
	plyOut.addVertexPositions(positions);
	plyOut.addVertexColors(colors);
	plyOut.write(filename, happly::DataFormat::Binary);
}


/**
 * A method to create a heatmap of distances between two point clouds.
 * @param source Source point cloud as Eigen matrix (3xN)
 * @param target Target point cloud as Eigen matrix (3xM)
 * @param zero_threshold Threshold for perfect alignment (default is 0.01f). Decrease if needed.
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr createHeatmap(
	const Eigen::MatrixXd& source,
	const Eigen::MatrixXd& target,
	float zero_threshold = 0.01f)
{
	//Use KDTree to get distances from source to target
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = eigenToPCL(target);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);

	std::vector<float> distances(source.cols());
	bool valid_distances = false;
	float min_dist = std::numeric_limits<float>::max();
	float max_dist = std::numeric_limits<float>::lowest();

	// First pass: compute distances and find min/max
	for (int i = 0; i < source.cols(); ++i) {
		Eigen::Vector3d pt = source.col(i);
		std::vector<int> indices(1);
		std::vector<float> dists(1);

		if (kdtree.nearestKSearch(pcl::PointXYZ(pt.x(), pt.y(), pt.z()), 1, indices, dists) > 0) {
			distances[i] = std::sqrt(dists[0]);
			min_dist = std::min(min_dist, distances[i]);
			max_dist = std::max(max_dist, distances[i]);
			valid_distances = true;
		}
	}

	if (!valid_distances) {
		std::cerr << "Error: No valid distances computed" << std::endl;
		return std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	}

	// Adjust range to avoid division by zero
	float range = max_dist - min_dist;
	if (range < 1e-6) {
		range = 1.0f;
	}

	// Create colored point cloud
	auto colored_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

	for (int i = 0; i < source.cols(); ++i) {
		pcl::PointXYZRGB pt;
		pt.x = source(0, i);
		pt.y = source(1, i);
		pt.z = source(2, i);

		// Handle perfect alignment
		if (distances[i] < zero_threshold) {
			pt.r = 255;
			pt.g = 255;
			pt.b = 255;
			colored_cloud->push_back(pt);
			continue;
		}

		// Normalize distance to [0,1] range
		float normalized = (distances[i] - min_dist) / range;

		// Blue to Red gradient 
		if (normalized < 0.5f) {
			// Blue gradient 
			float t = normalized / 0.5f; // 0 to 1 as normalized goes from 0 to 0.5
			pt.r = static_cast<uint8_t>(255 * t);        // 0 → 255
			pt.g = static_cast<uint8_t>(255 * t);        // 0 → 255
			pt.b = 255;                                  // 255 stays
		}
		else {
			// Red gradient: transition from purple (magenta) to bright red as intensity increases
			float t = (normalized - 0.5f) / 0.5f; // 0 to 1 as normalized goes from 0.5 to 1
			pt.r = 255;                                  // stays 255
			pt.g = static_cast<uint8_t>(255 * (1.0f - t)); // 255 → 0
			pt.b = static_cast<uint8_t>(255 * (1.0f - t)); // 255 → 0
		}


		colored_cloud->push_back(pt);
	}

	std::cout << "Heatmap range: " << min_dist << "m to " << max_dist << "m\n";
	return colored_cloud;
}


/**
 * A method to create a threshold-based heatmap of distances between two point clouds.
 * @param source Source point cloud as Eigen matrix (3xN)
 * @param target Target point cloud as Eigen matrix (3xM)
 * @param zero_threshold Perfect alignment threshold in mm (default 0.5mm)
 * @return Colored point cloud with discrete colors based on deviation ranges
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr createThresholdHeatmap(
	const Eigen::MatrixXd& source,
	const Eigen::MatrixXd& target,
	float zero_threshold = 1.0f)  // All values in mm
{
	// Convert to PCL and build KD-tree
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = eigenToPCL(target);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);

	// Create output cloud
	auto colored_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

	// Color definitions (RGB)
	const std::array<uint8_t, 3> BLUE = { 0, 0, 255 };
	const std::array<uint8_t, 3> YELLOW = { 255, 255, 0 };
	const std::array<uint8_t, 3> ORANGE = { 255, 165, 0 };
	const std::array<uint8_t, 3> RED = { 255, 0, 0 };
	const std::array<uint8_t, 3> WHITE = { 255, 255, 255 };

	for (int i = 0; i < source.cols(); ++i) {
		pcl::PointXYZRGB pt;
		pt.x = source(0, i);
		pt.y = source(1, i);
		pt.z = source(2, i);

		// Get nearest neighbor distance
		Eigen::Vector3d src_pt = source.col(i);
		std::vector<int> indices(1);
		std::vector<float> dists(1);

		if (kdtree.nearestKSearch(pcl::PointXYZ(src_pt.x(), src_pt.y(), src_pt.z()), 1, indices, dists) > 0) {
			float distance_mm = std::sqrt(dists[0]) * 1000;  // Convert meters to mm

			// Apply color based on thresholds
			if (distance_mm <= zero_threshold) {
				// Perfect alignment 0-1mm (green)
				pt.r = BLUE[0];
				pt.g = BLUE[1];
				pt.b = BLUE[2];
			}
			else if (distance_mm <= 5.0f) {
				// 0.5-1.5mm deviation (yellow)
				pt.r = YELLOW[0];
				pt.g = YELLOW[1];
				pt.b = YELLOW[2];
			}
			else if (distance_mm <= 7.5f) {
				// 1.5-2.5mm deviation (orange)
				pt.r = ORANGE[0];
				pt.g = ORANGE[1];
				pt.b = ORANGE[2];
			}
			else {
				// >10mm deviation (red)
				pt.r = RED[0];
				pt.g = RED[1];
				pt.b = RED[2];
			}
		}
		else {
			// No correspondence found (gray). A 
			pt.r = 128;
			pt.g = 128;
			pt.b = 128;
		}

		colored_cloud->push_back(pt);
	}

	return colored_cloud;
}


/**
 * A method to visualize a heatmap of chamfer distances between two point clouds.
 * This method creates a PCL visualizer and displays the heatmap as a colored point cloud.
 * @param heatmap_cloud Point cloud with RGB colors representing distances
 */
void visualizeHeatmap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& heatmap_cloud) {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Chamfer Distance Heatmap"));
	viewer->setBackgroundColor(0.05, 0.05, 0.05);

	viewer->addPointCloud<pcl::PointXYZRGB>(heatmap_cloud, "heatmap");

	viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "heatmap");

	//viewer->addCoordinateSystem(1.0);

	viewer->initCameraParameters();

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
}

// ============== COMPUTE SIMILARITY ==============

/**
 * A method to compute the Hausdorff Distance between two point clouds.
 * This method uses a KdTree to find the nearest neighbor in the target cloud for each point in the source cloud.
 * The Hausdorff Distance is computed as the maximum distance from each source point to its nearest target point.
 * The LOWER this number, the better the alignment.
 * The main limitation of Hausdorff Distance is that it is very sensitive to outliers.
 * @param source Source point cloud as Eigen matrix (3xN)
 * @param target Target point cloud as Eigen matrix (3xM)
 * @return Hausdorff Distance as a double
*/
double computeHausdorffDistance(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target) {
	auto target_cloud = eigenToPCL(target);
	pcl::KdTreeFLANN<pcl::PointXYZ> target_tree;
	target_tree.setInputCloud(target_cloud);

	double max_dist = 0.0;
	for (int i = 0; i < source.cols(); ++i) {
		Eigen::Vector3d pt = source.col(i);
		std::vector<int> indices(1);
		std::vector<float> dists(1);
		if (target_tree.nearestKSearch(pcl::PointXYZ(pt.x(), pt.y(), pt.z()), 1, indices, dists) > 0) {
			max_dist = std::max(max_dist, static_cast<double>(std::sqrt(dists[0])));
		}
	}
	return max_dist;
}

/** 
 * A method to compute the Chamfer Distance between two point clouds.
 * This method uses a KdTree to find the nearest neighbor in the target cloud for each point in the source cloud.
 * The Chamfer Distance is computed as the average distance from each source point to its nearest target point.
 * The LOWER this number, the better the alignment.
 * @param source Source point cloud as Eigen matrix (3xN)
 * @param target Target point cloud as Eigen matrix (3xM)
 * @return Chamfer Distance as a double
*/
double computeChamferDistance(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target) {
	auto target_cloud = eigenToPCL(target);
	pcl::KdTreeFLANN<pcl::PointXYZ> target_tree;
	target_tree.setInputCloud(target_cloud);

	double total_dist = 0.0;
	for (int i = 0; i < source.cols(); ++i) {
		Eigen::Vector3d pt = source.col(i);
		std::vector<int> indices(1);
		std::vector<float> dists(1);
		if (target_tree.nearestKSearch(pcl::PointXYZ(pt.x(), pt.y(), pt.z()), 1, indices, dists) > 0) {
			total_dist += std::sqrt(dists[0]);
		}
	}
	return total_dist / source.cols();
}

/**
 * A method to compute the Root Mean Square Error (RMSE) between two point clouds.
 * This method uses a KdTree to find the nearest neighbor in the target cloud for each point in the source cloud.
 * The RMSE is computed as the square root of the average squared distances.
 * The LOWER this number, the better the alignment.
 * @param source Source point cloud as Eigen matrix (3xN)
 * @param target Target point cloud as Eigen matrix (3xM)
 * @return RMSE as a double
*/
double computeRMSE(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target) {
	pcl::KdTreeFLANN<pcl::PointXYZ> target_tree;
	target_tree.setInputCloud(eigenToPCL(target));

	double squared_sum = 0.0;
	for (int i = 0; i < source.cols(); ++i) {
		Eigen::Vector3d pt = source.col(i);
		std::vector<int> indices(1);
		std::vector<float> dists(1);
		if (target_tree.nearestKSearch(pcl::PointXYZ(pt.x(), pt.y(), pt.z()), 1, indices, dists) > 0) {
			squared_sum += dists[0];
		}
	}
	return std::sqrt(squared_sum / source.cols());
}

/**
 * A method to compute the Jaccard Index between two point clouds.
 * Essentially checks what percentage of voxels are in the intersection of the two point clouds.
 * The HIGHER this number, the better the alignment.
 * This method voxelizes the point clouds and computes the Jaccard Index based on the intersection and union of voxels.
 * @param source Source point cloud as Eigen matrix (3xN)
 * @param target Target point cloud as Eigen matrix (3xM)
 * @param voxel_size Size of each voxel (default is 0.01f)
 * @return Jaccard Index as a double
*/
double computeJaccardIndex(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target, float voxel_size = 0.01f) {

	auto hash_vector3d = [](const Eigen::Vector3d& v) {
		return std::hash<double>()(v.x()) ^ std::hash<double>()(v.y()) ^ std::hash<double>()(v.z());
		};

	std::unordered_set<Eigen::Vector3d, decltype(hash_vector3d)> source_voxels(source.cols(), hash_vector3d);
	std::unordered_set<Eigen::Vector3d, decltype(hash_vector3d)> target_voxels(target.cols(), hash_vector3d);

	auto voxelize = [voxel_size](const Eigen::MatrixXd& cloud, auto& voxel_set) {
		for (int i = 0; i < cloud.cols(); ++i) {
			Eigen::Vector3d voxel = (cloud.col(i) / voxel_size).array().floor() * voxel_size;
			voxel_set.insert(voxel);
		}
		};

	voxelize(source, source_voxels);
	voxelize(target, target_voxels);

	size_t intersection = 0;
	for (const auto& voxel : source_voxels) {
		if (target_voxels.count(voxel)) intersection++;
	}

	size_t union_size = source_voxels.size() + target_voxels.size() - intersection;
	return union_size > 0 ? static_cast<double>(intersection) / union_size : 0.0;
}

/**
 * A method to compute all similarity metrics between two point clouds.
 */
SimilarityMetrics computeSimilarityMetrics(const Eigen::MatrixXd& source,
	const Eigen::MatrixXd& target,
	float voxel_size = 0.01f) {
	return {
		computeHausdorffDistance(source, target),
		computeChamferDistance(source, target),
		computeRMSE(source, target),
		computeJaccardIndex(source, target, voxel_size)
	};
}

/**
 * A method to write similarity metrics to a JSON file.
 * This method uses nlohmann::json to create a JSON object and writes it to the specified file.
 * @param metrics SimilarityMetrics object containing computed metrics
 * @param filename Output JSON file name
 */
void writeMetricsToJSON(const SimilarityMetrics& metrics, const std::string& filename) {

	nlohmann::json j;
	j["hausdorff_distance"] = metrics.hausdorff_distance;
	j["chamfer_distance"] = metrics.chamfer_distance;
	j["root_mean_square_error"] = metrics.rmse;
	j["jaccard_index"] = metrics.jaccard_index;

	std::ofstream file(filename);
	if (file.is_open()) {
		file << j.dump(4);
		file.close();
	}
	else {
		std::cerr << "Failed to open file for writing: " << filename << std::endl;
	}
}

// ============= RUNNING THE PROGRAM =============

int main() {

	if (NFD_Init() != NFD_OKAY) {
		std::cerr << "Failed to initialize nativefiledialog." << std::endl;
		return 1;
	}

	auto start = std::chrono::high_resolution_clock::now();

	std::string sourcePath = getFilePathNFD("ply");
	if (sourcePath.empty()) return 1;
	happly::PLYData mesh1(sourcePath);

	std::string targetPath = getFilePathNFD("ply");
	if (targetPath.empty()) return 1;
	happly::PLYData mesh2(targetPath);

	

	//happly::PLYData mesh1("C:\\Users\\satke569\\Documents\\ProgData\\CloudAlign\\CloudAlign\\source.ply");
	//happly::PLYData mesh1("C:\\Users\\satke569\\Documents\\ProgData\\CloudAlign\\CloudAlign\\fulion_Revoscan.ply");

	std::cout << "Loaded source, " << mesh1.getVertexPositions().size() << " points." << std::endl;

	//happly::PLYData mesh2("C:\\Users\\satke569\\Documents\\ProgData\\CloudAlign\\CloudAlign\\targetTrial1.ply");
	//happly::PLYData mesh2("C:\\Users\\satke569\\Documents\\ProgData\\CloudAlign\\CloudAlign\\source.ply");
	//happly::PLYData mesh2("C:\\Users\\satke569\\Documents\\ProgData\\CloudAlign\\CloudAlign\\fulion_EinScan.ply");


	std::cout << "Loaded target, " <<  mesh2.getVertexPositions().size() << " points." << std::endl;

	auto plyPoints1 = mesh1.getVertexPositions();
	Unit unit1 = detectUnit(plyPoints1);
	size_t n1 = plyPoints1.size();
	Eigen::MatrixXd source(3, n1);   //Source matrix created
	double scale1 = (unit1 == Unit::MILLIMETERS) ? 0.001 : 1.0;
	for (size_t i = 0; i < n1; ++i) {
		const auto& p = plyPoints1[i];
		source.col(i) << p[0] * scale1, p[1] * scale1, p[2] * scale1;
	}

	auto plyPoints2 = mesh2.getVertexPositions();
	Unit unit2 = detectUnit(plyPoints2);
	size_t n2 = plyPoints2.size();
	Eigen::MatrixXd target(3, n2);  //Target matrix created
	double scale2 = (unit2 == Unit::MILLIMETERS) ? 0.001 : 1.0;
	for (size_t i = 0; i < n2; ++i) {
		const auto& p = plyPoints2[i];
		target.col(i) << p[0] * scale2, p[1] * scale2, p[2] * scale2;
	}

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Loading ply files took: " << duration.count() << " ms" << std::endl;
	std::cout << "\n=================\n\n" << std::endl;

	// ========== TESTING PCA ALIGNMENT ==========

	//Eigen::Vector3d test_translation(33.0, -61.5, 21.5);
	//Eigen::Vector3d test_rotation(130, 30, 210);  // XYZ Euler angles in degrees
	//Eigen::Vector3d test_scale(1.4, 1.4, 1.4);  //Keep X, Y and Z the same for alignment. 

	//// Apply test transformation to target
	//Eigen::MatrixXd transformed_target = applyTestTransform(
	//	target,
	//	test_translation,
	//	test_rotation,
	//	test_scale
	//);

	//Easy backup to avoid test manipulations
	Eigen::MatrixXd transformed_target = target;

	// ========== VISUALIZATION AND ALIGNMENT ==========

	// Visualize before alignment
	visualize(source, transformed_target, "Original Source and Target");

	start = std::chrono::high_resolution_clock::now();

	PCA(source, transformed_target);

	end = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Alignment by Princial Component Analysis took: " << duration.count() << " ms" << std::endl;
	std::cout << "\n=================\n\n" << std::endl;

	visualize(source, transformed_target, "After PCA");

	start = std::chrono::high_resolution_clock::now();

	Eigen::MatrixXd icp_aligned;
	Eigen::Matrix4f icp_transform = performICP(source, transformed_target, icp_aligned);

	end = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Alignment by Iterative Closest Point took: " << duration.count() << " ms" << std::endl;
	std::cout << "\n=================\n\n" << std::endl;


	visualize(icp_aligned, transformed_target, "After ICP");  // Final alignment

	Heatmap config;
	config.min_value = 0.0f;      
	config.max_value = 2.0f;      
	config.zero_threshold = 0.01f; 

	auto heatmapCloud = createHeatmap(icp_aligned, transformed_target, 0.005f);
	visualizeHeatmap(heatmapCloud);

	std::string savePath = getSaveFilePathNFD();
	if (!savePath.empty()) {
		saveHeatmapToPLY(heatmapCloud, savePath);
		std::cout << "Saved heatmap to: " << savePath << std::endl;
	}
	else {
		std::cout << "Save cancelled." << std::endl;
	}


	auto thresholdHeatmapCloud = createThresholdHeatmap(icp_aligned, transformed_target, 0.5f);
	visualizeHeatmap(thresholdHeatmapCloud);
	std::string thresholdSavePath = getSaveFilePathNFD();
	if (!thresholdSavePath.empty()) {
		saveHeatmapToPLY(thresholdHeatmapCloud, thresholdSavePath);
		std::cout << "Saved threshold heatmap to: " << thresholdSavePath << std::endl;
	}
	else {
		std::cout << "Save cancelled." << std::endl;
	}



	//std::cout << "ICP transformation:\n" << icp_transform << std::endl;

	auto metrics = computeSimilarityMetrics(icp_aligned, transformed_target, 0.005f);
	std::cout << "Similarity Metrics:\n\n"
		<< "Hausdorff Distance: " << metrics.hausdorff_distance << " m\n"
		<< "Chamfer Distance: " << metrics.chamfer_distance << " m\n"
		<< "RMSE: " << metrics.rmse << " m\n"
		<< "Jaccard Index: " << metrics.jaccard_index << std::endl;

	// Save to JSON file on disk
	std::string jsonPath = getSaveFilePathNFD("metrics.json");
	if (!jsonPath.empty()) {
		writeMetricsToJSON(metrics, jsonPath);
		std::cout << "\nMetrics saved to: " << jsonPath << std::endl;
	}

	NFD_Quit();

	return 0;
}