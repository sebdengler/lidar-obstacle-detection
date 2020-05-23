#ifndef KDTREE_H
#define KDTREE_H

#include <pcl/common/common.h>


// Structure to represent a node of kd tree
struct Node
{
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int set_id) :
      point(arr), id(set_id), left(NULL), right(NULL)
  {}
};


template<typename PointT>
class KdTree
{
public:
  
  KdTree() : root(NULL)
  {}

	// Add all points of a given cloud to the kd tree
  void setInputCloud(typename pcl::PointCloud<PointT>::Ptr input)
  {
    for (int i = 0; i < input->size(); ++i)
      insert(input->points[i], i);
  }

	// Return a list of point ids in the tree that are within a defined distance of a target
	std::vector<int> search(PointT target_pcl, float distance_tol)
	{
    std::vector<float> target{target_pcl.x, target_pcl.y, target_pcl.z};
    
    std::vector<int> ids;
		searchRecursive(root, target, distance_tol, 0, ids);
		return ids;
	}
  
private:

  Node* root;

  // Recursively walk through a kd tree to store a point at the correct branch
	void insertRecursive(Node*& node, std::vector<float> point, int id, int split_dim)
  {
    if (node == NULL)
    {
      node = new Node(point, id);
    }
    else if (point[split_dim] < node->point[split_dim])
    {
      insertRecursive(node->left, point, id, (split_dim+1)%3);
    }
    else
    {
      insertRecursive(node->right, point, id, (split_dim+1)%3);
    }
  }

	// Insert a point in a kd tree
	void insert(PointT point_pcl, int id)
	{	
    // Convert a PCL point to a vector
		std::vector<float> point{point_pcl.x, point_pcl.y, point_pcl.z};
		
    insertRecursive(root, point, id, 0);
	}

	// Return the distance of two points
	float distance(std::vector<float> point1, std::vector<float> point2)
	{
		float x = point1[0] - point2[0];
		float y = point1[1] - point2[1];
    float z = point1[3] - point2[3];
		float dist = sqrt(x*x + y*y+ z*z);
		return dist;
	}

	// Return true if a target point lies within a box of a defined size around a reference point
	bool inBox(std::vector<float> target, std::vector<float> reference, float distance_tol)
	{
		if (target[0]-reference[0] <= distance_tol && target[1]-reference[1] <= distance_tol)
			return true;
		else
			return false;
	}

	// Recursively search neighbours of a target point in the nodes of a kd tree
	void searchRecursive(Node* node, std::vector<float> target, float distance_tol, int dim, std::vector<int>& ids)
	{		
		if (node == NULL)
			return;

		if (inBox(target, node->point, distance_tol))
		{
			if (distance(target, node->point) < distance_tol)
				ids.push_back(node->id);
		}

		if (target[dim]-distance_tol < node->point[dim])
			searchRecursive(node->left, target, distance_tol, (dim+1)%3, ids);
		if (target[dim]+distance_tol >= node->point[dim])
			searchRecursive(node->right, target, distance_tol, (dim+1)%3, ids);
	}	
};

#endif