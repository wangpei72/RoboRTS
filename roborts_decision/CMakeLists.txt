project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        nav_msgs
        geometry_msgs
        actionlib
        roborts_common
        roborts_msgs
        roborts_costmap
        dynamic_reconfigure
        )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
# For clion symbol updates
include_directories(
        ../../../devel/include
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
        DecisionProtoSrc
        DecisionProtoHds
        ${ProtoFiles}
        )
generate_dynamic_reconfigure_options(
        ../config/roborts_dynamic_cfg.cfg
)

catkin_package()

add_executable(behavior_test_node
        ${DecisionProtoHds}
        ${DecisionProtoSrc}
        example_behavior/escape_behavior.h
        example_behavior/back_boot_area_behavior.h
        example_behavior/chase_behavior.h
        example_behavior/patrol_behavior.h
        example_behavior/search_behavior.h
        example_behavior/attack_behavior.h
        behavior_test.cpp
        executor/chassis_executor.h
        executor/chassis_executor.cpp
        executor/gimbal_executor.h
        executor/gimbal_executor.cpp
        ../roborts_common/include/interface/dynamic_reconfigure_interface.h
        ../roborts_common/include/interface/dynamic_reconfigure_interface.cpp
        )

target_link_libraries(behavior_test_node
        PRIVATE
        roborts_costmap
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        )

add_dependencies(behavior_test_node
        roborts_msgs_generate_messages ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})