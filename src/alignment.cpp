#include <iostream>
#include <unordered_map>
#include <functional>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cmath>

// Define a custom hash for std::pair<float, float>
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator ()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        // Combine the two hash values
        return h1 ^ (h2 << 1);  // Simple way to combine hashes
    }
};

// Function to round float to a given number of decimal places
float round_to_decimal_places(float value, int decimal_places) {
    float scale = std::pow(10.0f, decimal_places);
    return std::round(value * scale) / scale;
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("/home/olia/automacon/scripts/map_processor/build/table_scene_lms400_inliers.pcd", *cloud);

    // какое округление использовать, точность 10 см
    const int accuracy = 2;
    
    std::unordered_map<std::pair<float, float>, float, pair_hash> min_z_map;
    for (const auto& point : cloud->points) {
        // округлили x и y, чтобы были точки с примерно одинаковыми x, y
        std::pair<float, float> xy_pair(round_to_decimal_places(point.x, accuracy), round_to_decimal_places(point.y, accuracy));

        if (min_z_map.find(xy_pair) == min_z_map.end()) {
            min_z_map[xy_pair] = point.z;
        } else {
            min_z_map[xy_pair] = std::min(min_z_map[xy_pair], point.z);
        }
    }

    // Update the cloud points by subtracting the minimum z for each (x, y) pair
    for (auto& point : cloud->points) {
        std::pair<float, float> xy_pair(round_to_decimal_places(point.x, accuracy), round_to_decimal_places(point.y, accuracy));
        point.z = point.z - min_z_map[xy_pair];
    }

    // Visualize the point cloud
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.addPointCloud(cloud, "cloud");

    pcl::io::savePCDFileBinaryCompressed("cloud_z.pcd", *cloud);


    while (!viewer.wasStopped()) {
        viewer.spinOnce();  // Handle events and update the viewer window
    }

    return 0;
}
