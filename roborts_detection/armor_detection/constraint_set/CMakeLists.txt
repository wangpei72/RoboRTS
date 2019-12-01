project(constraint_set)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
        ConstraintSetProtoSrc
        ConstraintSetProtoHds
        ${ProtoFiles}
        )

include_directories(../)
link_directories(../)

find_package(Eigen3 REQUIRED)
add_library(constraint_set
        SHARED
        ${ConstraintSetProtoSrc}
        ${ConstraintSetProtoHds}
        constraint_set.cpp
        light_blob.cpp light_blob.h armor_box.h armor_box.cpp ../classifier.cpp ../classifier.h)

add_library(detection::constraint_set ALIAS constraint_set)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        detection::tool
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Eigen3_LIBRARIES}
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${catkin_INCLUDE_DIRS}
        )
