# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_STANDARD 17)

find_package(Rock)
rock_init(robot_model 0.1)
rock_standard_layout()
