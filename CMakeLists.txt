cmake_minimum_required(VERSION 2.8.3)
project(ubr_calibration)

find_package(Boost REQUIRED system thread)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)  # visualization only

find_library(tinyxml_library tinyxml)
if (tinyxml_library)
  message (STATUS "Looking for libtinyxml - found")
  set(tinyxml_LIBRARIES ${tinyxml_library})
endif ()
find_path(tinyxml_include_dirs NAMES tinyxml.h PATH_SUFFIXES tinyxml)
if (NOT tinyxml_include_dirs)
   message (STATUS "Looking for tinyxml/tinyxml.hpp or tinyxml/tinyxml.h - not found.")
endif ()

find_package(orocos_kdl)
find_package(Ceres)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    cv_bridge
    geometry_msgs
    kdl_parser
    message_generation
    pcl_ros
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    ubr_msgs
)

add_message_files(FILES CalibrationData.msg)
generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS}
                            ${catkin_INCLUDE_DIRS}
                            ${CERES_INCLUDES}
                            ${OpenCV_INCLUDE_DIRS}  # visualization only
                            ${PCL_INCLUDE_DIRS}
                            ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS}) # this is a hack, will eventually be unneeded once orocos-kdl is fixed

# Always build capture
add_executable(capture src/capture.cpp
                       src/chain_manager.cpp
                       src/led_finder.cpp)
target_link_libraries(capture ${Boost_LIBRARIES}
                              ${catkin_LIBRARIES}
                              ${CERES_LIBRARIES}
                              ${OpenCV_LIBRARIES}  # visualization only
                              ${PCL_LIBRARIES})
add_dependencies(capture ubr_calibration_gencpp
                         ubr_msgs_gencpp)

add_executable(capture_checkerboards src/capture_checkerboards.cpp
                                     src/chain_manager.cpp
                                     src/checkerboard_finder.cpp)
target_link_libraries(capture_checkerboards ${Boost_LIBRARIES}
                                            ${catkin_LIBRARIES}
                                            ${CERES_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            ${PCL_LIBRARIES})
add_dependencies(capture_checkerboards ubr_calibration_gencpp
                                       ubr_msgs_gencpp)

if (${Ceres_FOUND})
add_executable(calibrate src/calibrate.cpp
                         src/update_urdf.cpp)
target_link_libraries(calibrate ${Boost_LIBRARIES}
                                ${catkin_LIBRARIES}
                                ${CERES_LIBRARIES}
                                ${tinyxml_LIBRARIES}
                                ${orocos_kdl_LIBRARIES})
add_dependencies(calibrate ubr_calibration_gencpp)

add_subdirectory(test)

install(TARGETS capture calibrate capture_checkerboards
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
else (${Ceres_FOUND})
  MESSAGE(WARNING "Could not locate Ceres. ubr_calibration will not be built.")
endif(${Ceres_FOUND})
