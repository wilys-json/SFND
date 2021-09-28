#ifndef RANSAC3D_H
#define RANSAC3D_H

#include "processPointClouds.h"

// Modify from ransac2d.cpp
template<typename PointT>
class RansacSegmentation {

	public:
		RansacSegmentation() {};
		~RansacSegmentation() {};
		void setMaxIterations(int maxIter) { maxIterations = maxIter; };
		void setDistanceThreshold(float distTol) { distanceTol = distTol; };
		std::unordered_set<int> segment(typename pcl::PointCloud<PointT>::Ptr cloud);

	private:
		int maxIterations = 0;
		float distanceTol = 0.0;

};
#endif
