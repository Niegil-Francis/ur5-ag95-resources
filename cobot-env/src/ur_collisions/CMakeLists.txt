add_executable(ur_collisions ur_collisions.cpp)
target_link_libraries(ur_collisions ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} pthread)