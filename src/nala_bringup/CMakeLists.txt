cmake_minimum_required(VERSION 3.8)
project(nala_bringup NONE)   # no C/C++ code, only installs files

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# The central config folder is at the repo root (../../config).
# With `colcon build --symlink-install` the installed files are links to it,
# so an edited file works without a rebuild. A new file needs a rebuild.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
