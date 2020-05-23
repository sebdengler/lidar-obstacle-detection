#ifndef PROCESSPOINTCLOUDS_H
#define PROCESSPOINTCLOUDS_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include "segmentation/ransac2d.h"
#include "clustering/kdtree.h"
#include "clustering/euclideanClustering.h"


template<typename PointT>
class ProcessPointClouds
{
public:
  
  ProcessPointClouds();
  ~ProcessPointClouds();

  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);
  
  typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filter_res, Eigen::Vector4f min_point, Eigen::Vector4f max_point);
  
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float distance_threshold);
  
  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance, int min_size, int max_size);
  
  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
  BoxQ BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);
  
  void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);
  
  typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);
  std::vector<boost::filesystem::path> streamPcd(std::string data_path);

private:
  
  bool kPrintInfo = false;

};

#endif