cmake_minimum_required(VERSION 3.5)
project(laser_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(agv_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(teaserpp REQUIRED)
find_package(Boost 1.58 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features kdtree visualization)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)
# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
# 设置依赖库链接目录
link_directories(${PCL_LIBRARY_DIRS})
# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS})


include_directories(
        include
        include/global_localization
        include/laser_odometry
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()

add_library(
        localization_lib
        src/localization/localization.cpp
        src/error_estimater/error_estimater.cpp
        src/localization/UKF.cpp
        src/global_localization/bbs_localization.cpp
        src/global_localization/global_localization.cpp
        src/laser_odometry/laser_odometry.cpp
        src/error_estimater/voxelset.cpp
        src/localization/tools.cpp
)
ament_target_dependencies(localization_lib rclcpp  visualization_msgs tf2_ros tf2 std_srvs nav_msgs sensor_msgs pcl_conversions)
target_link_libraries(
        localization_lib
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        ${OpenCV_LIBS}
)


add_executable(laser_localization src/main.cpp)
target_link_libraries(laser_localization
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        ${OpenCV_LIBS}
        localization_lib
)
install(TARGETS
        laser_localization
        DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        )
### test

add_executable(test_laser_odom test/test_laser_odom.cpp)
target_link_libraries(test_laser_odom
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        localization_lib
        )

add_executable(test_global_localization test/test_global_localization.cpp)
target_link_libraries(test_global_localization
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        localization_lib
        ${OpenCV_LIBS}
        )

add_executable(test_error test/test_error.cpp)
target_link_libraries(test_error
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        localization_lib
        ${OpenCV_LIBS}
        )

add_executable(read_map test/read_map.cpp)
target_link_libraries(read_map ${PCL_LIBRARIES})

add_executable(odom test/test_odom.cpp)
target_link_libraries(odom ${PCL_LIBRARIES})
ament_target_dependencies(odom rclcpp  visualization_msgs tf2_ros tf2 nav_msgs sensor_msgs pcl_conversions)

add_executable(kitti test/test_kitti.cpp)
target_link_libraries(kitti ${PCL_LIBRARIES})
ament_target_dependencies(kitti rclcpp  visualization_msgs tf2_ros tf2 nav_msgs sensor_msgs pcl_conversions)
