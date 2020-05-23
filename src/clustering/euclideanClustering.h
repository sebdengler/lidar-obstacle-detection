#ifndef EUCLIDEAN_CLUSTERING_H
#define EUCLIDEAN_CLUSTERING_H

#include "kdtree.h"


// Recursively find nearby points
template<typename PointT>
void proximity(std::vector<std::pair<PointT, bool>>& points_proc,
  int index, pcl::PointIndices& cluster, KdTree<PointT>* tree, float cluster_tolerance)
{
  points_proc[index].second = true;
  cluster.indices.push_back(index);
  std::vector<int> nearby = tree->search(points_proc[index].first, cluster_tolerance);
  // Recursively do proximity search for all nearby unprocessed points 
  for (auto it = nearby.begin(); it != nearby.end(); ++it)
  {
    if (!points_proc[*it].second)
      proximity<PointT>(points_proc, *it, cluster, tree, cluster_tolerance);
  }
}


// Extract clusters from a point cloud by making use of a kd tree
template<typename PointT>
std::vector<pcl::PointIndices> ExtractEuclideanClusters(
  typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree,
  float cluster_tolerance, int min_size, int max_size)
{
  std::vector<pcl::PointIndices> clusters;

  std::vector<std::pair<PointT, bool>> points_proc;
  for (auto point : *cloud)
    points_proc.push_back(std::make_pair(point, false));

  for (int i = 0; i < cloud->size(); ++i)
  {
    if (!points_proc[i].second)
    {
      pcl::PointIndices new_cluster;
      // Find all points in the neighbourhood of the current point
      proximity<PointT>(points_proc, i, new_cluster, tree, cluster_tolerance);
      // If the cluster size is larger than a defined minimum size and smaller
      // than a defined maximum size, add it to the list of clusters
      if (new_cluster.indices.size() >= min_size && new_cluster.indices.size() <= max_size)
        clusters.push_back(new_cluster);
    }
  }

  return clusters;
}


#endif