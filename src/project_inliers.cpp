#include <iostream>
#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("/home/olia/automacon/scripts/map_processor/build/table_scene_lms400_inliers.pcd", *cloud);
    

    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);


    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // viewer.addPointCloud(cloud, "cloud");
    viewer.addPointCloud(cloud_projected, "cloud_projected");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "cloud_projected");


    while (!viewer.wasStopped()) {
        viewer.spinOnce();  // Handle events and update the viewer window
    }


    return (0);
}
