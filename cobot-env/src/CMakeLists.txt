set(FLIGHT_DEPENDENCIES flight_dependencies/flight_ctrl.cpp 
						flight_dependencies/pid_linear.cpp 
						flight_dependencies/pid_rotate.cpp
	)

add_subdirectory(human_drone_plugins)
# add_subdirectory(human_drone_ctrl)