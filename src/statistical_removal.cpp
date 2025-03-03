#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZI>("/home/olia/automacon/scripts/map_processor/build/output_compressed.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    //   The number of neighbors to analyze for each point is set to 50
    sor.setMeanK(200);
    //   standard deviation multiplier to 1
    //  all points who have a distance larger than 1 standard deviation
    //  of the mean distance to the query point will be marked as outliers and removed
    sor.setStddevMulThresh(1.0);

    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    // pcl::PCDWriter writer;
    //   writer.write<pcl::PointXYZI> ("table_scene_lms400_inliers.pcd", *cloud_filtered, true); //true - добавить заголовок в файл
    
    pcl::io::savePCDFileBinaryCompressed("cloud_inliers.pcd", *cloud_filtered);

    //   Then, the filter is called with the same parameters, but with the output negated, to obtain the outliers (e.g., the points that were filtered)
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    pcl::io::savePCDFileBinaryCompressed("cloud_outliers.pcd", *cloud_filtered);

    return 0;
}
