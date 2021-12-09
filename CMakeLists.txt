cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pointcloud_skeleton_extractor)

find_package(PCL 1.2 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB CPP_FILES src/*.cpp)

add_executable(pointcloud_skeleton_extractor ${CPP_FILES})
target_link_libraries(pointcloud_skeleton_extractor ${PCL_LIBRARIES})
target_link_libraries (pointcloud_skeleton_extractor Eigen3::Eigen)
target_link_libraries(pointcloud_skeleton_extractor ${VTK_LIBRARIES})
