cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(MY_GRAND_PROJECT)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write_test pcd_write.cpp)
add_executable(customer_point_type_test customer_point_type.cpp)
add_executable(matrix_transform_test matrix_transform.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})
target_link_libraries(customer_point_type_test ${PCL_LIBRARIES})
target_link_libraries(matrix_transform_test ${PCL_LIBRARIES})