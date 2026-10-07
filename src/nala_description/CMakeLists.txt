cmake_minimum_required(VERSION 3.8)
project(nala_description NONE)   # no C/C++ code, only installs files

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
