cmake_minimum_required(VERSION 3.1)
project(pid_controller)

#common::pid_controller
add_library(pid_controller INTERFACE

        )

target_include_directories(alg_factory INTERFACE include/)
add_library(roborts_common::pid_controller ALIAS pid_controller)
