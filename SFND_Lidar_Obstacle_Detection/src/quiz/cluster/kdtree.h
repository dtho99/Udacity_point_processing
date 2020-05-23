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
	
		int level = 0;
		bool inserted = false;
		Node *newNodePtr = new Node(point, id);
		Node *currNodePtr;
		 	
		newNodePtr->id = id;
		newNodePtr->point[0] = point[0];
		newNodePtr->point[1] = point[1];			 
		//newNodePtr->point = point;
		newNodePtr->left = NULL;
		newNodePtr->right = NULL;

		if (root == NULL)
		{
			root = newNodePtr;

			return;
		}

		currNodePtr = root;

		while (inserted == false)
		{
			if ((level % 2) == 0)
			{
				// split x direction
				if (newNodePtr->point[0] < currNodePtr->point[0])
				{
					if (currNodePtr->left == NULL)
					{
						currNodePtr->left = newNodePtr;
						inserted = true;
					}
					else 
					{
						currNodePtr = currNodePtr->left;
					}
				}
				else
				{
					if (currNodePtr->right == NULL)
					{
						currNodePtr->right = newNodePtr;
						inserted = true;
					}
					else 
					{
						currNodePtr = currNodePtr->right;
					}
				}
			}
			else
			{
				// split y direction
				if (newNodePtr->point[1] < currNodePtr->point[1])
				{
					if (currNodePtr->left == NULL)
					{
						currNodePtr->left = newNodePtr;
						inserted = true;
					}
					else 
					{
						currNodePtr = currNodePtr->left;
					}
				}
				else
				{
					if (currNodePtr->right == NULL)
					{
						currNodePtr->right = newNodePtr;
						inserted = true;
					}
					else 
					{
						currNodePtr = currNodePtr->right;
					}
				}				
			}

			level++;
		}

		
	}

	void Proximity(std::vector<float> point, std::vector<int> &cluster)
	{
		

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		Node *currNodePtr;

		currNodePtr = root;
		int level = 0;

	    checkNode(target, currNodePtr, level, ids, distanceTol);
		
		return ids;
	}


	void checkNode(std::vector<float> target, Node * currNodePtr, int level, std::vector<int> &ids, float distanceTol)
	{


		if (currNodePtr == NULL)
			return;

//		printf("Entry (%d) (%4.1f), %4.1f,%4.1f,  %4.1f,%4.1f\n", 
//			level, distanceTol, 
//			target[0], target[1], currNodePtr->point[0], currNodePtr->point[1]);

		if ((level % 2) == 0)
		{
			// check x.
			if (fabs(currNodePtr->point[0] - target[0]) < distanceTol)
			{
				if (calcDistance(currNodePtr->point, target) < distanceTol)
				{
					ids.push_back(currNodePtr->id);
			
				}	

			    checkNode(target, currNodePtr->left, level + 1, ids, distanceTol);
			    checkNode(target, currNodePtr->right, level + 1, ids, distanceTol);								

			}
			else
			{
				/* code */
				if ((target[0] < currNodePtr->point[0]) && (currNodePtr->left != NULL))
				    checkNode(target, currNodePtr->left, level + 1, ids, distanceTol);
				else if ((target[0] >= currNodePtr->point[0]) && (currNodePtr->right != NULL))
				    checkNode(target, currNodePtr->right, level + 1, ids, distanceTol);				
			}	
			

				
		}
		else
		{
			// check y.
			if (fabs(currNodePtr->point[1] - target[1]) < distanceTol)
			{
				if (calcDistance(currNodePtr->point, target) < distanceTol)
				{
					ids.push_back(currNodePtr->id);
				}
			    checkNode(target, currNodePtr->left, level + 1, ids, distanceTol);
			    checkNode(target, currNodePtr->right, level + 1, ids, distanceTol);								

			}
			else
			{
				if ((target[1] < currNodePtr->point[1]) && (currNodePtr->left != NULL))
				    checkNode(target, currNodePtr->left, level + 1, ids, distanceTol);
				else if ((target[1] >= currNodePtr->point[1]) && (currNodePtr->right != NULL))
			    	checkNode(target, currNodePtr->right, level + 1, ids, distanceTol);
			}
		}
	

	}

	float calcDistance(std::vector<float>  pos1, std::vector<float>  pos2)
	{
		float delta_x = pos1[0] - pos2[0];
		float delta_y = pos1[1] - pos2[1];
		return (sqrt((delta_x * delta_x) + (delta_y * delta_y)));
	}
};




