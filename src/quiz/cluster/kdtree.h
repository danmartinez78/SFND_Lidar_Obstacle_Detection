/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	void insert(std::vector<float> point, int id)
	{
		// the function should create a new node and place correctly with in the root
		recurssive_insert(&root, 0, point, id);
	}

	void recurssive_insert(Node **node, int depth, std::vector<float> data, int id)
	{
		if (*node == NULL)
		{
			*node = new Node(data, id);
		}
		else
		{
			int split_dem = depth % 2; // 2D Tree
			Node **new_node;
			if (data[split_dem] >= ((*node)->point[split_dem]))
			{
				// greatear than => split right
				new_node = &((*node)->right);
			}
			else
			{
				// less than => split left
				new_node = &((*node)->left);
			}
			// reduction step
			recurssive_insert(new_node, depth + 1, data, id);
			
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// start at root
		recurssive_search(root, target, distanceTol, 0, ids);
		return ids;
	}

	void recurssive_search(Node *node, std::vector<float> target, float distanceTol, int depth, std::vector<int>& ids)
	{
		int split_dem = depth % 2; // 2D Tree
		if (node != NULL)
		{
			// check if passed node is within tol along both directions
			float d[2] = {(target[0] - node->point[0]), (target[1] - node->point[1])};
			if (std::fabs(d[0]) <= distanceTol && std::fabs(d[1]) <= distanceTol)
			{
				if (std::sqrt(std::pow(d[0], 2) + std::pow(d[1], 2) <= distanceTol))
				{
					ids.push_back(node->id);
				}
			}
			if (d[split_dem] < distanceTol)
			{
				recurssive_search(node->left, target, distanceTol, depth + 1, ids);
			}
			if (d[split_dem] > -distanceTol)
			{
				recurssive_search(node->right, target, distanceTol, depth + 1, ids);
			}
		}
	}
};
