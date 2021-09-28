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
    void Proximity(pcl::PointIndices& cluster, const int& id);

  public:
    EuclideanCluster3D() {};
    ~EuclideanCluster3D() { tree = nullptr; };
    void setInputCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud);
    void setKdTree(KdTree<PointT>* kdtree) { tree = kdtree; };
    void setMin(int minSize) { minClusterSize = minSize; };
    void setMax(int maxSize) { maxClusterSize = maxSize; };
    void setClusterTolerance(float clusterTol) { clusterTolerance = clusterTol; };
    void extract(std::vector<pcl::PointIndices>& clusterIndices);

};

#endif
