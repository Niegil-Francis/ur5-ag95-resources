add_executable(build_rviz_scene build_rviz_scene.cpp)
target_link_libraries(build_rviz_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})