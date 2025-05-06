# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.10)
project(robot_model
    VERSION 1.0
    DESCRIPTION "A libray which holds information on robot states")

set(CMAKE_CXX_STANDARD 17)

find_package(Rock)
rock_init()
rock_standard_layout()
