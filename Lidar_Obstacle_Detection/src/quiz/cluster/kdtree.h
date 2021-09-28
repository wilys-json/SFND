/* \author Aaron Brown */
// Quiz on implementing kd tree

// #include "../../render/render.h" // Comment out to include in main program
#ifndef CUSTOMKDTREE_H_
#define CUSTOMKDTREE_H_
#include "../../processPointClouds.h"

// Structure to represent node of kd tree

template<typename PointT>
class Node
{
public:
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename PointT>
class KdTree
{
private:
	Node<PointT>* root;

	public:

	KdTree()
	: root(NULL)
	{}

	KdTree(typename pcl::PointCloud<PointT>::Ptr cloud) : root(NULL)
	{
		for (int i = 0; i < cloud->points.size(); ++ i) {
			insert(cloud->points[i], i);
		}
	}

	~KdTree()
	{
		delete root;
	}

	void insert(PointT point, int id)
	{
		if (root == nullptr) { root = new Node<PointT>(point, id); return; }
		Node<PointT>* currentNode = root;
		insertHelper(&currentNode, point, 0, id);
	}

	void insertHelper(Node<PointT>** currentNode, std::vector<float> point, int depth, int id) {
		if (*currentNode == nullptr) { *currentNode = new Node<PointT>(point, id); return; }
		if (point[depth%2] <= (*currentNode)->point[depth%2])
			insertHelper(&((*currentNode)->left), point, depth+1, id);
		else
			insertHelper(&((*currentNode)->right), point, depth+1, id);
	}

	void insertHelper(Node<PointT>** currentNode, PointT point, int depth, int id) {
		if (*currentNode == nullptr) { *currentNode = new Node<PointT>(point, id); return; }
		float currentValue;
		float otherValue;
		switch(depth%3) {
			case 0:
				otherValue = point.x;
				currentValue = (*currentNode)->point.x;
			case 1:
				otherValue = point.y;
				currentValue = (*currentNode)->point.y;
			case 2:
				otherValue = point.z;
				currentValue = (*currentNode)->point.z;
		}

		if (otherValue <= currentValue)
			insertHelper(&((*currentNode)->left), point, depth+1, id);
		else
			insertHelper(&((*currentNode)->right), point, depth+1, id);
	}

	// return a list of point ids in the tree that are within distance of target

	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(ids, root, target, distanceTol, 0);
		return ids;
	}

	void searchHelper(std::vector<int>& ids, Node<std::vector<float>>* currentNode, std::vector<float> target, float distanceTol, int depth) {
		if (currentNode == nullptr) return;

		if (currentNode->point[0] <= (target[0] + distanceTol) &&
				currentNode->point[0] >= (target[0] - distanceTol) &&
				currentNode->point[1] <= (target[1] + distanceTol) &&
				currentNode->point[1] >= (target[1] - distanceTol)) {
			float distance = std::sqrt(
				std::pow((target[1] - currentNode->point[1]),2) +
				std::pow((target[0] - currentNode->point[0]),2)
			);
			if (distance <= distanceTol) ids.push_back(currentNode->id);
		}

		if ((target[depth%2] - distanceTol) < currentNode->point[depth%2])
			searchHelper(ids, currentNode->left, target, distanceTol, depth+1);
		if ((target[depth%2] + distanceTol) > currentNode->point[depth%2])
			searchHelper(ids, currentNode->right, target, distanceTol, depth+1);
	}

	void searchHelper(std::vector<int>& ids, Node<PointT>* currentNode, PointT target, float distanceTol, int depth) {
		if (currentNode == nullptr) return;

		float x, y, z, tx, ty, tz;
		x = currentNode->point.x;
		y = currentNode->point.y;
		z = currentNode->point.z;
		tx = target.x;
		ty = target.y;
		tz = target.z;

		if (x <= (tx + distanceTol) && x >= (tx - distanceTol) &&
				y <= (ty + distanceTol) && y >= (ty - distanceTol) &&
			  z <= (tz + distanceTol) && z >= (tz - distanceTol)) {
			float distance = std::sqrt(
				std::pow((tx - x),2) + std::pow((ty - y),2) + std::pow((tz - z),2)
			);
			if (distance <= distanceTol) ids.push_back(currentNode->id);
		}

		std::vector<float> currentValues{x,y,z};
		std::vector<float> targetValues{tx,ty,tz};

		if ((targetValues[depth%2] - distanceTol) < currentValues[depth%2])
			searchHelper(ids, currentNode->left, target, distanceTol, depth+1);
		if ((targetValues[depth%2] + distanceTol) > currentValues[depth%2])
			searchHelper(ids, currentNode->right, target, distanceTol, depth+1);
	}

	int size() {
		int i = 0;
		auto currentNode = root;
		return traverse(root, i);
	}

	int traverse(Node<PointT>* node, int size) {
		if (node == nullptr) return size;
		++size;
		size = traverse(node->left, size);
		size = traverse(node->right, size);
		return size;
	}

};

#endif
