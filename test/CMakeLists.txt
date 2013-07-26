catkin_add_gtest(test_error_blocks test_error_blocks.cpp)
target_link_libraries(test_error_blocks ${Boost_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        ${CERES_LIBRARIES})
