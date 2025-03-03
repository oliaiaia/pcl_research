cmake_minimum_required(VERSION 3.8)
project(map_processor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Найти зависимости
# find_package(nlohmann_json REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)

# Работа с PCL, фильтрация
add_executable(compress_to_binary src/compress_to_binary.cpp)
target_link_libraries(compress_to_binary PRIVATE ${PCL_LIBRARIES})

add_executable (passthrough src/passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})

add_executable (statistical_removal src/statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (project_inliers src/project_inliers.cpp)
target_link_libraries (project_inliers ${PCL_LIBRARIES})

add_executable (voxel src/voxel_grid.cpp)
target_link_libraries (voxel ${PCL_LIBRARIES})

add_executable (alignment src/alignment.cpp)
target_link_libraries (alignment ${PCL_LIBRARIES})