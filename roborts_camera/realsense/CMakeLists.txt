project(realsense)
find_package(realsense2)
find_package(OpenCV 3 REQUIRED)
add_library(realsense_driver SHARED ./realsense.cpp)
target_link_libraries(realsense_driver
        ${realsense2_LIBRARY}
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        )
target_include_directories(realsense_driver
        PRIVATE
        ${realsense2_INCLUDE_DIRS}
        )
target_include_directories(realsense_driver
        PRIVATE
        ${catkin_INCLUDE_DIRS}
        )
target_include_directories(realsense_driver
        PRIVATE
        ${OpenCV_INCLUDE_DIRECTORIES}
        )

add_library(driver::realsense_driver
        ALIAS
        realsense_driver
        )

