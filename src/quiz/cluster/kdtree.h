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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root		
		recurssive_insert(&root, 0, point, id);
	}

	void recurssive_insert(Node** node, int depth, std::vector<float> data, int id)
	{
		if (*node == NULL){
			*node = new Node(data, id);
		}
		else{
			int split_dem = depth % 2; // 2D Tree
			Node **new_node;
			if (data[split_dem] > (*node)->point[split_dem]){
				// greatear than => split right
				new_node = &((*node)->right);
			}
			else{
				// greatear than => split left
				new_node = &((*node)->left);
			}
			// reduction step
			recurssive_insert(new_node, depth+1, data, id);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




