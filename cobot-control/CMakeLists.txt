cmake_minimum_required(VERSION 2.8.3)
project(cobot-control)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              std_msgs
              sensor_msgs 
              geometry_msgs 
              cv_bridge
              )
catkin_package(CATKIN_DEPENDS roscpp 
                              std_msgs 
                              sensor_msgs 
                              geometry_msgs
                              cv_bridge
                              )

link_directories(include
                 ${GAZEBO_LIBRARY_DIRS}
                 ${Boost_LIBRARIES}
                 ${PROTOBUF_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

include_directories( 
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    )

add_subdirectory(scripts)