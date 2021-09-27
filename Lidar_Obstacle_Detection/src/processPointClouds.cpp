// PCL lib Functions for processing point clouds
#include <chrono>
#include "processPointClouds.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

// RANSAC from quiz/ransac2d.cpp
template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));


	// For max iterations

	for (int i = 0; i < maxIterations; ++ i) {

		std::unordered_set<int> tempInliersResult;
		while (tempInliersResult.size() < 3)
			tempInliersResult.insert(rand() % cloud->points.size());
		// Randomly sample subset and fit line
		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		auto itr = tempInliersResult.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// Define A, B, C, D
		float A, B, C, D;
		A = ((y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1));
		B = ((z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1));
		C = ((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));
		D = (A * x1 + B * y1 + C * z1) * -1;
	// Measure distance between every point and fitted line
		for (int i = 0; i < cloud->points.size(); ++i) {
				if (tempInliersResult.count(i) > 0) continue;
		// If distance is smaller than threshold count it as inlier
				float dist = std::abs(
					(A * cloud->points[i].x) +
					(B * cloud->points[i].y) +
					(C * cloud->points[i].z) +
					D) / std::sqrt(std::pow(A, 2) + std::pow(B, 2) + std::pow(C, 2));
				if (dist <= distanceTol) tempInliersResult.insert(i);
		}
		if (tempInliersResult.size() > inliersResult.size())
			inliersResult = tempInliersResult;
	}
	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;

}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f& minPoint, Eigen::Vector4f& maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Apply Voxel Grid filter
    typename pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filtered);

    // Apply ROI Crop box filter
    typename pcl::PointCloud<PointT>::Ptr roi(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cropBox(true);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(filtered);
    cropBox.filter(*roi);

    // Remove roof: from tutorial
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, 1, 1));
    roof.setInputCloud(roi);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto& index : indices)
      inliers->indices.push_back(index);


    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (roi);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*roi);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roi;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{

    typename pcl::PointCloud<PointT>::Ptr planeCloud (new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new typename pcl::PointCloud<PointT>);


    pcl::ExtractIndices<PointT> extract;

    for (auto& index : inliers->indices)
      planeCloud->points.push_back(cloud->points[index]);

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // // Comment out this block to time
    // auto startTime = std::chrono::steady_clock::now();

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // // Create the segmentation object
    // pcl::SACSegmentation<PointT> seg;
    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (maxIterations);
    // seg.setDistanceThreshold (distanceThreshold);
    //
    //
    // // Segment the largest planar component from the remaining cloud
    // seg.setInputCloud (cloud);
    // seg.segment (*inliers, *coefficients);
    // if (inliers->indices.size () == 0)
    // {
    //   std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    // }

    auto inliers_set = Ransac(cloud, maxIterations, distanceThreshold);
    for (auto& index : inliers_set) inliers->indices.push_back(index);

    // // Comment out this block to time
    // auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    // Codes from PCL documentation
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      for (const auto& idx : it->indices)
        cloud_cluster->push_back ((*cloud)[idx]); //*
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      clusters.push_back(std::move(cloud_cluster));
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    // std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
