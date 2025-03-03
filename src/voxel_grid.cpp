#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main ()
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("/home/olia/automacon/scripts/map_processor/build/table_scene_lms400_inliers.pcd", *cloud); 

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.05f, 0.05f, 0.05f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

//   pcl::PCDWriter writer;
//   writer.write ("cloud_downsampled.pcd", *cloud_filtered, 
//          Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_xyz);
pcl::io::savePCDFileBinaryCompressed("cloud_downsampled.pcd", *cloud_xyz);

  return 0;
}
