cmake_minimum_required(VERSION 2.8.3)
project(roborts_executor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        dynamic_reconfigure
        actionlib
        geometry_msgs
        roborts_common
        roborts_msgs
        roborts_decision
        roscpp
        tf
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
        ../config/roborts_dynamic_cfg.cfg
)

catkin_package()

include_directories(
#        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(executor_action_node
        executor_action/pid_controller_executor_action_server.cpp
        executor_action/pid_controller_executor_action_server.h
        ../roborts_common/include/interface/dynamic_reconfigure_interface.cpp

)


## Specify libraries to link a library or executable target against
target_link_libraries(executor_action_node
        ${catkin_LIBRARIES}
        )