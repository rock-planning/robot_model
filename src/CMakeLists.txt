project(robot_model)

set(ROBOT_MODEL_VERSION 0.1)
set(TARGET_NAME ${PROJECT_NAME})

INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)
pkg_check_modules(BASE_TYPES REQUIRED base-types)
pkg_search_module(BASE_LOGGING REQUIRED base-logging)
pkg_check_modules(KDL_PARSER REQUIRED kdl_parser)
pkg_check_modules(URDF REQUIRED urdfdom)
pkg_check_modules(URDFHEADER REQUIRED urdfdom_headers)
pkg_check_modules(SRDF REQUIRED srdfdom)
pkg_check_modules(OCTOMAP REQUIRED octomap)
pkg_check_modules(KINEMATICS REQUIRED kinematics_library)
pkg_check_modules(COLLISION_DETECTION REQUIRED collision_detection)

set(ROBOTMODEL_LIB_INCLUDES ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} 
                            ${BASE_TYPES_INCLUDE_DIRS} ${BASE_LOGGING_INCLUDE_DIRS} ${KDL_PARSER_INCLUDE_DIRS} 
                            ${URDF_INCLUDE_DIRS} ${URDFHEADER_INCLUDE_DIRS} ${SRDF_INCLUDE_DIRS}
                            ${OCTOMAP_INCLUDE_DIRS} ${KINEMATICS_INCLUDE_DIRS} ${COLLISION_DETECTION_INCLUDE_DIRS})

set(ROBOTMODEL_LINK_DIRS    ${BASE_TYPES_LIBRARY_DIRS} ${BASE_LOGGING_LIBRARY_DIRS} ${KDL_PARSER_LIBRARY_DIRS} 
                            ${URDF_LIBRARY_DIRS} ${URDFHEADER_LIBRARY_DIRS} ${SRDF_LIBRARY_DIRS}
                            ${OCTOMAP_LIBRARY_DIRS} ${KINEMATICS_LIBRARY_DIRS} ${COLLISION_DETECTION_LIBRARY_DIRS}
)

set(ROBOTMODEL_DEP_LIBS    ${BASE_TYPES_LIBRARIES} ${BASE_LOGGING_LIBRARIES} ${KDL_PARSER_LIBRARIES}
                           ${URDF_LIBRARIES} ${URDFHEADER_LIBRARIES} ${SRDF_LIBRARIES} ${OCTOMAP_LIBRARIES}
                           ${KINEMATICS_LIBRARIES} ${COLLISION_DETECTION_LIBRARIES})

include_directories(${ROBOTMODEL_LIB_INCLUDES})

link_directories(${ROBOTMODEL_LINK_DIRS})

add_library(robot_model SHARED
            ${CMAKE_CURRENT_LIST_DIR}/RobotModel.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotLink.cpp
            ${CMAKE_CURRENT_LIST_DIR}/HelperFunctions.cpp
)

target_link_libraries(robot_model ${ROBOTMODEL_DEP_LIBS})

set(PROJECT_DESCRIPTION "A libray which holds information on robot states")
# now we add all the required libraries in PKGCONFIG_REQUIRES macro
list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES base-logging)
list(APPEND PKGCONFIG_REQUIRES kdl_parser)
list(APPEND PKGCONFIG_REQUIRES urdfdom)
list(APPEND PKGCONFIG_REQUIRES urdfdom_headers)
list(APPEND PKGCONFIG_REQUIRES srdfdom)
list(APPEND PKGCONFIG_REQUIRES octomap)
list(APPEND PKGCONFIG_REQUIRES kinematics_library)
list(APPEND PKGCONFIG_REQUIRES collision_detection)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY ../include/ DESTINATION include/)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 




