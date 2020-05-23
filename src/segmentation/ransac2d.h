#ifndef RANSAC2D_H
#define RANSAC2D_H

#include <unordered_set>
#include <pcl/common/common.h>


template<typename PointT>
pcl::PointIndices::Ptr RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float distance_tol)
{
	std::unordered_set<int> inliers_result;
	int num_outliers = cloud->size();
	srand(time(NULL));

  // For a defined number of maximum iterations
	for (int it = 0; it < max_iterations; ++it)
	{
		// Randomly choose 3 point in the point cloud
    std::unordered_set<int> new_inliers;
		while (new_inliers.size() < 3)
			new_inliers.insert(rand() % cloud->size());

    // Get the coordinates of the 3 chosen points
		auto p_it = new_inliers.begin();
		float x1 = (*cloud)[*p_it].x;
		float y1 = (*cloud)[*p_it].y;
		float z1 = (*cloud)[*p_it].z;
		p_it++;
		float x2 = (*cloud)[*p_it].x;
		float y2 = (*cloud)[*p_it].y;
		float z2 = (*cloud)[*p_it].z;
		p_it++;
		float x3 = (*cloud)[*p_it].x;
		float y3 = (*cloud)[*p_it].y;
		float z3 = (*cloud)[*p_it].z;

    // Get the plane's parameters
		float a = ((y2-y1)*(z3-z1)) - ((z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1)) - ((x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1)) - ((y2-y1)*(x3-x1));
		float d = -((a*x1) + (b*y1) + (c*z1));

		int index = 0;
		int new_num_outliers = 0;

    // For all points in the cloud
		for (auto c_it = cloud->begin(); c_it != cloud->end(); ++c_it)
		{
			// Measure the distance between the point and the plane
      float dist = fabs(a*c_it->x + b*c_it->y + c*c_it->z + d) / sqrt(a*a + b*b + c*c);
      // If the distance is smaller than a defined
      // threshold add it to the set of inliers
      if (dist < distance_tol)
			{
				new_inliers.insert(index);
			}
			else
			{
				new_num_outliers++;
				// Break if this model already contains more outliers
				// than the current best fit
				if (new_num_outliers > num_outliers)
					break;
			}
			index++;
		}
		
    // If the number of inliers for this plane is larger than
    // the current best fit use this one as a new best fit
		if (new_inliers.size() > inliers_result.size())
		{	
			inliers_result = new_inliers;
			num_outliers = new_num_outliers;
		}
	}

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (auto it = inliers_result.begin(); it != inliers_result.end(); ++it)
    inliers->indices.push_back(*it);

	std::sort(inliers->indices.begin(), inliers->indices.end()); 
	
  // Return the indicies of inliers from the plane with the most inliers
	return inliers;
}

#endif