cmake_minimum_required(VERSION 3.1)
project(roborts_interface)

#common::roborts_interface
add_library(roborts_interface INTERFACE)

target_include_directories(roborts_interface INTERFACE include/)