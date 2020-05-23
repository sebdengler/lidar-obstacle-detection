// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filter_res, Eigen::Vector4f min_point, Eigen::Vector4f max_point)
{

    // Time filtering process
    auto start_time = std::chrono::steady_clock::now();

    // Filter the point cloud by specifying a bounding box
    pcl::CropBox<PointT> box_filter;
    box_filter.setInputCloud(cloud);
    box_filter.setMin(min_point);
    box_filter.setMax(max_point);
    box_filter.filter(*cloud);

    // Downsample the point cloud
    pcl::VoxelGrid<PointT> downsample_filter;
    downsample_filter.setInputCloud(cloud);
    downsample_filter.setLeafSize(filter_res, filter_res, filter_res);
    downsample_filter.filter(*cloud);

    // Filter the point cloud by removing all points that come from the car's roof
    pcl::CropBox<PointT> roof_filter;
    std::vector<int> indices;
    roof_filter.setInputCloud(cloud);
    roof_filter.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof_filter.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof_filter.filter(indices);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto i : indices)
      inliers->indices.push_back(i);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    if (kPrintInfo)
    {
      auto end_time = std::chrono::steady_clock::now();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      std::cout << "filtering took " << elapsed_time.count() << " milliseconds" << std::endl;
    }

    return cloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr c_inliers(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr c_outliers(new pcl::PointCloud<PointT>);

    auto indIt = inliers->indices.begin();
    for (int it = 0; it != cloud->size(); ++it)
    {
        if (*indIt == it)
        {
            c_inliers->push_back((*cloud)[it]);
            ++indIt;
        }
        else
        {
            c_outliers->push_back((*cloud)[it]);
        }
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> seg_result(c_outliers, c_inliers);
    return seg_result;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float distance_threshold)
{
    // Time segmentation process
    auto start_time = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers = RansacPlane<PointT>(cloud, max_iterations, distance_threshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> seg_result = SeparateClouds(inliers, cloud);

    if (kPrintInfo)
    {
      auto end_time = std::chrono::steady_clock::now();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      std::cout << "plane segmentation took " << elapsed_time.count() << " milliseconds" << std::endl;
    }

    return seg_result;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
  // Find bounding box for one of the clusters
  PointT min_point, max_point;
  pcl::getMinMax3D(*cluster, min_point, max_point);
  
  Box box;
  box.x_min = min_point.x;
  box.y_min = min_point.y;
  box.z_min = min_point.z;
  box.x_max = max_point.x;
  box.y_max = max_point.y;
  box.z_max = max_point.z;
  
  return box;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance, int min_size, int max_size)
{
    // Time clustering process
    auto start_time = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree<PointT>* tree = new KdTree<PointT>();
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices = ExtractEuclideanClusters<PointT>(cloud, tree, cluster_tolerance, min_size, max_size);

    for (auto c_it = cluster_indices.begin(); c_it != cluster_indices.end(); ++c_it)
    {
      typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
      for (auto p_it = c_it->indices.begin(); p_it != c_it->indices.end(); ++p_it)
        cluster->push_back((*cloud)[*p_it]);
      cluster->width = cluster->size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(cluster);
    }

    if (kPrintInfo)
    {
      auto end_time = std::chrono::steady_clock::now();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      std::cout << "clustering took " << elapsed_time.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    }

    return clusters;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  
  if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  if (kPrintInfo)
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;
  
  return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string data_path)
{
  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{data_path}, boost::filesystem::directory_iterator{});
  
  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());
  
  return paths;
}