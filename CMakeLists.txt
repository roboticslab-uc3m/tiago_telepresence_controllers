cmake_minimum_required(VERSION 3.0)

project(tiago_telepresence_controllers LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    kdl_parser
    urdf
    message_generation
)

find_package(orocos_kdl REQUIRED)

add_service_files(
    FILES
    ArmMotion.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/controller_base.cpp
                            src/head_tp_controller.cpp
                            src/torso_tp_controller.cpp
                            src/gripper_tp_controller.cpp
                            src/arm_tp_controller.cpp
                            src/command_buffer.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/tp_controller_switcher.py
                 scripts/arm_motion.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES tiago_telepresence_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
