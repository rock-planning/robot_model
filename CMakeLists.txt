cmake_minimum_required(VERSION 3.10)
project(robot_model VERSION 0.1 DESCRIPTION "A library which holds information on robot states")

set(CMAKE_CXX_STANDARD 14)

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

include(FindPkgConfig)
pkg_check_modules(OROCOS_KDL REQUIRED orocos_kdl)
pkg_check_modules(KDL_PARSER REQUIRED kdl_parser)
pkg_check_modules(URDF REQUIRED urdfdom)
pkg_check_modules(URDFHEADER REQUIRED urdfdom_headers)
pkg_check_modules(SRDF REQUIRED srdfdom)
pkg_check_modules(OCTOMAP REQUIRED octomap)
pkg_check_modules(KINEMATICS REQUIRED kinematics_library)
pkg_check_modules(COLLISION_DETECTION REQUIRED collision_detection)
pkg_check_modules(BASE_TYPES REQUIRED base_types)
pkg_search_module(BASE_LOGGING REQUIRED base_logging)

# Include directories and libraries
set(ROBOTMODEL_LIB_INCLUDES 
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OROCOS_KDL_INCLUDE_DIRS} 
    ${KDL_PARSER_INCLUDE_DIRS}
    ${URDF_INCLUDE_DIRS} 
    ${URDFHEADER_INCLUDE_DIRS} 
    ${SRDF_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS} 
    ${KINEMATICS_INCLUDE_DIRS} 
    ${COLLISION_DETECTION_INCLUDE_DIRS}
    ${BASE_TYPES_INCLUDE_DIRS} 
    ${BASE_LOGGING_INCLUDE_DIRS}
)

set(ROBOTMODEL_DEP_LIBS 
    Threads::Threads
    Eigen3::Eigen 
    Boost::system Boost::filesystem
    ${OROCOS_KDL_LIBRARIES} 
    ${KDL_PARSER_LIBRARIES} 
    ${URDF_LIBRARIES} 
    ${URDFHEADER_LIBRARIES} 
    ${SRDF_LIBRARIES}
    ${OCTOMAP_LIBRARIES} 
    ${KINEMATICS_LIBRARIES} 
    ${COLLISION_DETECTION_LIBRARIES}
    ${BASE_TYPES_LIBRARY} 
    ${BASE_LOGGING_LIBRARY}
)

# Library setup
add_library(robot_model SHARED
    src/RobotModel.cpp
    src/RobotLink.cpp
    src/HelperFunctions.cpp
)

target_include_directories(robot_model PUBLIC ${ROBOTMODEL_LIB_INCLUDES})
target_link_libraries(robot_model PUBLIC ${ROBOTMODEL_DEP_LIBS})

# PKGCONFIG setup
list(APPEND PKGCONFIG_REQUIRES 
    base_types base_logging orocos_kdl kdl_parser urdfdom urdfdom_headers srdfdom 
    octomap kinematics_library collision_detection
)
list(JOIN PKGCONFIG_REQUIRES " " PKGCONFIG_REQUIRES_STR)

configure_file("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}.pc.in" "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc" @ONLY)

# Install rules
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME} DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)
