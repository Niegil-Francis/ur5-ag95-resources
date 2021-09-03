cmake_minimum_required(VERSION 2.8.3)
project(cobot-env)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED
            COMPONENTS
            roscpp
            rospy
            std_msgs
            gazebo_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs gazebo_msgs gazebo_ros
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    )
include_directories( 
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_subdirectory(scripts/build_scene)
add_subdirectory(src)