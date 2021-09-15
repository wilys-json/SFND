/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		if (root == nullptr) { root = new Node(point, id); return; }
		Node* currentNode = root;
		insertHelper(&currentNode, point, 0, id);
	}

	void insertHelper(Node** currentNode, std::vector<float> point, int depth, int id) {
		if (*currentNode == nullptr) { *currentNode = new Node(point, id); return; }
		if (point[depth%2] <= (*currentNode)->point[depth%2])
			insertHelper(&((*currentNode)->left), point, depth+1, id);
		else
			insertHelper(&((*currentNode)->right), point, depth+1, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(ids, root, target, distanceTol, 0);
		return ids;
	}

	int search(std::vector<float> target)
	{
		return searchHelper(root, target, 0);
	}

	int searchHelper(Node* currentNode, std::vector<float> target, int depth)
	{
		if (target[0] == currentNode->point[0] && target[1] == currentNode->point[1])
			return currentNode->id;
		if (target[depth%2] <= currentNode->point[depth%2])
			return searchHelper(currentNode->left, target, depth+1);
		if (target[depth%2] > currentNode->point[depth%2])
			return searchHelper(currentNode->right, target, depth+1);
		return -1;
	}

	void searchHelper(std::vector<int>& ids, Node* currentNode, std::vector<float> target, float distanceTol, int depth) {
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

};
