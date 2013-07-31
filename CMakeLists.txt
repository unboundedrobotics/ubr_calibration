cmake_minimum_required(VERSION 2.8.3)
project(ubr_calibration)

find_package(Boost REQUIRED system thread)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)  # visualization only
find_package(catkin REQUIRED message_generation actionlib roscpp orocos_kdl kdl_parser ubr_msgs
                             std_msgs sensor_msgs geometry_msgs rosbag pcl_ros)

add_message_files(FILES CalibrationData.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package()

# cmake hack for finding gflags
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/lib/x86_64-linux-gnu")

find_package(Ceres)
if (${Ceres_FOUND})

include_directories(include ${Boost_INCLUDE_DIRS}
                            ${catkin_INCLUDE_DIRS}
                            ${CERES_INCLUDES}
                            ${OpenCV_INCLUDE_DIRS}  # visualization only
                            ${PCL_INCLUDE_DIRS})

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

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate ${Boost_LIBRARIES}
                                ${catkin_LIBRARIES}
                                ${CERES_LIBRARIES})
add_dependencies(calibrate ubr_calibration_gencpp)

add_subdirectory(test)

install(TARGETS capture calibrate
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
else (${Ceres_FOUND})
  MESSAGE(WARNING "Could not locate Ceres. ubr_calibration will not be built.")
endif(${Ceres_FOUND})
