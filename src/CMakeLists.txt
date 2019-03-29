project(robot_model)

set(ROBOT_MODEL_VERSION 0.1)
set(TARGET_NAME ${PROJECT_NAME})

INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
pkg_check_modules(kdl_parser REQUIRED kdl_parser)
pkg_check_modules(SRDF REQUIRED srdfdom)
pkg_check_modules(OCTOMAP REQUIRED octomap)
pkg_check_modules(KINEMATICS REQUIRED kinematics_library)
pkg_check_modules(COLLISION_DETECTION REQUIRED collision_detection)


include_directories(${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${kdl_parser_INCLUDE_DIRS} ${SRDF_INCLUDE_DIRS} 
		    ${OCTOMAP_INCLUDE_DIRS} ${COLLISION_DETECTION_INCLUDE_DIRS} ${KINEMATICS_INCLUDE_DIRS} )
link_directories(${kdl_parser_LIBRARY_DIRS})

add_library(robot_model SHARED
            ${CMAKE_CURRENT_LIST_DIR}/RobotModel.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotLink.cpp
            ${CMAKE_CURRENT_LIST_DIR}/HelperFunctions.cpp
)

target_link_libraries(robot_model  ${kdl_parser_LIBRARIES} ${SRDF_LIBRARIES} ${orocos_kdl_LIBRARIES} 
				    octomap abstract_collision_detection ${COLLISION_DETECTION_LIBRARIES} ${KINEMATICS_LIBRARIES})

set(PKGCONFIG_CFLAGS "${PKFCONFIG_CFLAGS} -I${COLLISION_DETECTION_INCLUDE_DIRS};${KINEMATICS_INCLUDE_DIRS}")
CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY ../include/ DESTINATION include/)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 




