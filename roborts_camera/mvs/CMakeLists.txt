
project( mvs_driver)


find_library(MVS_LIBRARIES libMvCameraControl.so /opt/MVS/Samples_LinuxSDK/lib/64)

include_directories(/opt/MVS/Samples_LinuxSDK/include)

add_library( mvs_driver SHARED ./mvs_driver.cpp)
target_include_directories(mvs_driver PRIVATE /opt/MVS/Samples_LinuxSDK/include)
target_include_directories(mvs_driver PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(mvs_driver ${OpenCV_LIBRARIES} ${MVS_LIBRARIES} ${catkin_LIBRARIES})

add_library(driver::mvs_driver ALIAS mvs_driver)



