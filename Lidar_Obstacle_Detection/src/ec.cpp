#include "ec.h"

// Recursive Function to find nearby points in the same cluster
template<typename PointT>
void EuclideanCluster3D<PointT>::Proximity(pcl::PointIndices& cluster, const int& id)
{

    cluster.indices.push_back(id); // Add point to cluster
    processed[id] = true;  // Mark point as processed

    // Find nearby points within Distance Tolerance
    std::vector<int> nearbyIds = tree->search(cloud->points[id], clusterTolerance);
    for (auto& nearbyId : nearbyIds) {
      // If nearby point has not been processed
      if (!processed[nearbyId])
        // Call recursive Proximity
        EuclideanCluster3D::Proximity(cluster, nearbyId);
    }
}

template<typename PointT>
void EuclideanCluster3D<PointT>::setInputCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud)
{
  cloud = inputCloud;
  processed.assign(cloud->points.size(), false);
}

// Clustering algorithm using euclidean distance
template<typename PointT>
void EuclideanCluster3D<PointT>::extract(std::vector<pcl::PointIndices>& clusterIndices)
{

  // Iterate through each point
  int i = 0;
  while(i < cloud->points.size()) {
      // If point has been proceed, go to next point
      if (processed[i]) { ++i; continue; }
      // Create new cluster
      pcl::PointIndices cluster;
      // Call recursive Proximity function
      Proximity(cluster, i);
      // Add cluster to output vector
      if (cluster.indices.size() >= minClusterSize &&
          cluster.indices.size() <= maxClusterSize)
             clusterIndices.push_back(cluster);
      else for(auto& index : cluster.indices) processed[index] = false;
      ++i;
  }

}
