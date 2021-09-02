############# Plugin for simulating flight #############
add_library(plugin_flight SHARED
    plugin_flight.cpp
    pid_controller.cpp
)

target_link_libraries(plugin_flight
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

set_target_properties(plugin_flight
    PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

############# Plugin for simulating altimeter #############
add_library(plugin_altimeter SHARED
    plugin_altimeter.cpp
)

target_link_libraries(plugin_altimeter
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
)

set_target_properties(plugin_altimeter
    PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)