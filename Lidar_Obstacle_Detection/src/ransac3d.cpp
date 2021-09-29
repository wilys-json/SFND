#include "ransac3d.h"


template<typename PointT>
std::unordered_set<int> RansacSegmentation<PointT>::segment(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // For `maxIterations` number of time
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
    for (int j = 0; j < cloud->points.size(); j++) {
        if (tempInliersResult.count(j) > 0) continue;
    // If distance is smaller than threshold count it as inlier
        float dist = std::abs(
          (A * cloud->points[j].x) +
          (B * cloud->points[j].y) +
          (C * cloud->points[j].z) +
          D) / std::sqrt(A*A + B*B + C*C);
        if (dist <= distanceTol) tempInliersResult.insert(j);
    }
    if (tempInliersResult.size() > inliersResult.size())
      inliersResult = tempInliersResult;
  }
  // Return indicies of inliers from fitted line with most inliers
  return inliersResult;

}
