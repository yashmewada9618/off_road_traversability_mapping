cmake_minimum_required(VERSION 3.8)
project(costmap_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED system serialization filesystem thread)
find_package(pcl_msgs REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)

set(BUILD_SHARED_LIBS ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(${EIGEN3_DEFINITIONS})

set(dependencies
    rclcpp
    nav_msgs
    sensor_msgs
    std_msgs
    geometry_msgs
    visualization_msgs
    Eigen3
    pcl_conversions)

include_directories(include include/${PROJECT_NAME} ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

add_executable(customCostmap src/customCostmap.cpp)

target_link_libraries(customCostmap ${PCL_LIBRARIES} ${Boost_LIBRARIES})

ament_target_dependencies(customCostmap ${dependencies})

# target_include_directories( customCostmap PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> $<INSTALL_INTERFACE:include>)
install(TARGETS customCostmap DESTINATION lib/${PROJECT_NAME})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(customScan src/customScan.cpp)
target_link_libraries(customScan ${PCL_LIBRARIES})
ament_target_dependencies(customScan ${dependencies})
install(TARGETS customScan DESTINATION lib/${PROJECT_NAME})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
