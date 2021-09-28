#ifndef CUSTOMEC_H_
#define CUSTOMEC_H_

#include "quiz/cluster/kdtree.h"
#include "processPointClouds.h"

template<typename PointT>
class EuclideanCluster3D {

private:
  KdTree<PointT>* tree;
  typename pcl::PointCloud<PointT>::Ptr cloud;
  int minClusterSize = 0;
  int maxClusterSize = 0;
  float clusterTolerance = 0.0;
  std::vector<bool> processed;
  std::vector<std::vector<int>> clusters;


    // Recursive Function to find nearby points in the same cluster
    void Proximity(pcl::PointIndices& cluster, const int& id)
    {

        cluster.indices.push_back(id); // Add point to cluster
        processed[id] = true;  // Mark point as processed

        // Find nearby points within Distance Tolerance
        std::vector<int> nearbyIds = tree->search(cloud->points[id], clusterTolerance);
        for (auto& nearbyId : nearbyIds) {
          // If nearby point has not been processed
          if (!processed[nearbyId])
            // Call recursive Proximity
            Proximity(cluster, nearbyId);
        }
    }

  public:

  EuclideanCluster3D() {};
  ~EuclideanCluster3D() { tree = nullptr; };
  void setInputCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud)
  {
    cloud = inputCloud;
    processed.assign(cloud->points.size(), false);
  };
  void setKdTree(KdTree<PointT>* kdtree) { tree = kdtree; };
  void setMin(int minSize) { minClusterSize = minSize; };
  void setMax(int maxSize) { maxClusterSize = maxSize; };
  void setClusterTolerance(float clusterTol) { clusterTolerance = clusterTol; };

  // Clustering algorithm using euclidean distance
  void extract(std::vector<pcl::PointIndices>& clusterIndices)
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

};

#endif
