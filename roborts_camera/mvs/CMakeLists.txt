
project( mvs_driver)


find_library(MVS_LIBRARIES libMvCameraControl.so /opt/MVS/Samples_LinuxSDK/lib/64)

include_directories(/opt/MVS/Samples_LinuxSDK/include)

add_library( msv_driver SHARED ./mvs_driver.cpp)
target_include_directories(msv_driver PRIVATE /opt/MVS/Samples_LinuxSDK/include)
target_include_directories(msv_driver PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(msv_driver ${OpenCV_LIBRARIES} ${MVS_LIBRARIES} ${catkin_LIBRARIES})



