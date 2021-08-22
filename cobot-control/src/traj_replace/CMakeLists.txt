set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)


# add_executable(traj_replace traj_replace.cpp)
# target_link_libraries(traj_replace ${catkin_LIBRARIES})

add_executable(traj_replace_gui gui/gui.h gui/gui.cpp)
target_link_libraries(traj_replace_gui  ${catkin_LIBRARIES} Qt${QT_VERSION_MAJOR}::Widgets)