#include <boost/thread/thread.hpp>

#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"


void initCamera(CameraAngle set_angle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  viewer->setBackgroundColor(0, 0, 0);
    
  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;
    
  switch(set_angle)
  {
    case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
    case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
    case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
    case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if(set_angle!=FPS)
    viewer->addCoordinateSystem(1.0);
}



void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* point_processor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input,
               bool render_filtered, bool render_road, bool render_obstacles,
               bool render_clusters, bool render_boxes)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = point_processor->FilterCloud(cloud_input, 0.2, Eigen::Vector4f(-15,-6,-3,1), Eigen::Vector4f(15,6,2,1));
  if (render_filtered)
    renderPointCloud(viewer, filterCloud, "filterCloud");
  
  // Segement the ground plane
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_segmented = point_processor->SegmentPlane(filterCloud, 100, 0.15);
  if (render_obstacles)
    renderPointCloud(viewer, cloud_segmented.first, "obstacleCloud", Color(1,0,0));
  if (render_road)
    renderPointCloud(viewer, cloud_segmented.second, "roadCloud", Color(0,1,0));
  
  // Find clusters in the obstacle cloud
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters = point_processor->Clustering(cloud_segmented.first, 0.4, 10, 1000);
  int cluster_id = 0;
  for (auto cluster : cloud_clusters)
  {
    // std::cout << "cluster size ";
    // point_processor->numPoints(cluster);
    if (render_clusters)
      renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(cluster_id), Color(0,0,1));
    Box box = point_processor->BoundingBox(cluster);
    if (render_boxes)
      renderBox(viewer, box, cluster_id);
    ++cluster_id;
  }
}



int main (int argc, char** argv)
{
  std::cout << "starting PCL visualizer" << std::endl;

  // Initialize the PCL visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle set_angle = FPS;
  initCamera(set_angle, viewer);

  ProcessPointClouds<pcl::PointXYZI>* point_processor = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = point_processor->streamPcd("../data/pcd/data_1");
  auto stream_iterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input;

  while (!viewer->wasStopped())
  {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    
    // Load pcd and run obstacle detection process
    cloud_input = point_processor->loadPcd(stream_iterator->string());
    cityBlock(viewer, point_processor, cloud_input, false, true, false, true, true);
    
    // When all files were processed, start from beginning
    stream_iterator++;
    if(stream_iterator == stream.end())
      stream_iterator = stream.begin();
      
    viewer->spinOnce();
    // boost::this_thread::sleep(boost::posix_time::milliseconds(500));
  }
}