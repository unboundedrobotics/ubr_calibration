catkin_add_gtest(test_error_blocks test_error_blocks.cpp)
target_link_libraries(test_error_blocks ${Boost_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        ${CERES_LIBRARIES})

catkin_add_gtest(test_urdf_update test_update_urdf.cpp
                                  ../src/update_urdf.cpp)
target_link_libraries(test_urdf_update ${Boost_LIBRARIES}
                                       ${catkin_LIBRARIES}
                                       ${tinyxml_LIBRARIES})
