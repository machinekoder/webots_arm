cmake_minimum_required(VERSION 2.8.3)
project(webots_arm)

add_compile_options(-std=c++14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED
             COMPONENTS webots_ros
        cv_bridge
        roscpp
        tf
        tf2_ros
        tf2_kdl
        kdl_parser
        image_transport
        camera_info_manager)
find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)

catkin_package(INCLUDE_DIRS include DEPENDS
        webots_ros
        cv_bridge
        roscpp
        tf2_ros
        tf2_kdl
        kdl_parser
        orocos_kdl
        urdfdom_headers)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(joint_control
        src/joint_control.cpp
        )
add_dependencies(joint_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_control ${catkin_LIBRARIES})

add_executable(camera_control
        src/camera_control.cpp)
add_dependencies(camera_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_control ${catkin_LIBRARIES})

add_executable(pointcloud_control
        src/pointcloud_control.cpp)
add_dependencies(pointcloud_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_control ${catkin_LIBRARIES})

install(TARGETS joint_control camera_control pointcloud_control
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
         FILES_MATCHING
         PATTERN "*.h"
         PATTERN ".svn"
         EXCLUDE)

# FIXME Are these actually necessary?
# # Install config and launch directories
# install(
#   DIRECTORY
#   config
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   )

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest)
#  set(UTEST_SRC_FILES test/utest.cpp)
#
#  add_rostest_gtest(compliant_control_utest test/launch/utest.launch ${UTEST_SRC_FILES})
#  target_link_libraries(compliant_control_utest ${catkin_LIBRARIES} ${Boost_LIBRARIES} compliant_control)
#endif()
