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
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    kdl_parser
    message_generation
    nav_msgs
    pcl_ros
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    ubr_msgs
)

catkin_python_setup()

add_message_files(FILES CalibrationData.msg DepthCameraInfo.msg)
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

add_executable(calibrate_base src/calibrate_base.cpp)
target_link_libraries(calibrate_base ${Boost_LIBRARIES}
                                     ${catkin_LIBRARIES})

if (${Ceres_FOUND})

add_executable(calibrate src/calibrate.cpp
                         src/calibration_offset_parser.cpp
                         src/chain_manager.cpp
                         src/checkerboard_finder.cpp
                         src/led_finder.cpp
                         src/models.cpp)
target_link_libraries(calibrate ${Boost_LIBRARIES}
                                ${catkin_LIBRARIES}
                                ${CERES_LIBRARIES}
                                ${OpenCV_LIBRARIES}  # visualization only
                                ${tinyxml_LIBRARIES}
                                ${orocos_kdl_LIBRARIES})
add_dependencies(calibrate ubr_calibration_gencpp
                           ubr_msgs_gencpp)

add_subdirectory(test)

install(TARGETS calibrate calibrate_base
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

else (${Ceres_FOUND})

  MESSAGE(WARNING "Could not locate Ceres. ubr_calibration will not be built.")

endif(${Ceres_FOUND})
