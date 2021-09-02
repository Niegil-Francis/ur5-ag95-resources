set(FLIGHT_DEPENDENCIES ${PROJECT_SOURCE_DIR}/src/flight_dependencies/flight_ctrl.cpp 
						${PROJECT_SOURCE_DIR}/src/flight_dependencies/pid_linear.cpp 
						${PROJECT_SOURCE_DIR}/src/flight_dependencies/pid_rotate.cpp
	)

add_subdirectory(human_drone_plugins)
add_subdirectory(human_drone_ctrl)