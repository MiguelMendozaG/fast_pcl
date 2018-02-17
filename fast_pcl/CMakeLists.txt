cmake_minimum_required(VERSION 2.8)

project(fast_pcl)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main main.cpp)
target_link_libraries(main ${PCL_LIBRARIES})

add_executable(filter_voxel filter_voxel.cpp)
target_link_libraries(filter_voxel ${PCL_LIBRARIES})

#add_executable(pcd_files_headers pcd_files_headers.cpp)
#target_link_libraries(pcd_files_headers ${PCL_LIBRARIES})

