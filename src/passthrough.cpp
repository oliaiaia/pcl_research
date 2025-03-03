#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string input_file = "/home/olia/automacon/scripts/map_processor/build/output_compressed.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        PCL_ERROR("Ошибка при чтении PCD файла.\n");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "Cloud before filtering: " << std::endl;

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    // фильтруем по z
    pass.setFilterFieldName("z");
    // от и до
    pass.setFilterLimits(0.0, 1.0);
    // pass.setNegative (true);
    pass.filter(*cloud_filtered);

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.addPointCloud(cloud, "cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
    viewer.addPointCloud(cloud_filtered, "cloud_filtered");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "cloud_filtered");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(); // Обработка событий и обновление окна визуализатора
    }
    return 0;
}
