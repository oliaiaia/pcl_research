#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>

bool compress_pcd_file(const std::string& input_file, const std::string& output_file) {
    // Создаем объект для облака точек
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Читаем несжатый файл PCD
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_file, *cloud) == -1) {
        PCL_ERROR("Ошибка при чтении PCD файла.\n");
        return false;
    }

    // Сохраняем облако точек в сжатом бинарном формате PCD
    pcl::io::savePCDFileBinaryCompressed(output_file, *cloud);

    std::cout << "Облако точек успешно сохранено в сжатом формате." << std::endl;
    return true;
}

int main() {
    std::string input_file = "/home/olia/automacon/scripts/map_processor/src/map.pcd";
    std::string output_file = "output_compressed.pcd";

    // Вызов функции для сжатия файла
    if (!compress_pcd_file(input_file, output_file)) {
        std::cerr << "Произошла ошибка при обработке файла." << std::endl;
        return -1;
    }

    return 0;
}
