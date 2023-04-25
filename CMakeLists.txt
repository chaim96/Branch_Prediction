cmake_minimum_required(VERSION 3.5)
project(HW1)

set(CMAKE_CXX_STANDARD 11)

include_directories(.)

add_executable(HW1
        bp.cpp
        bp_api.h
        bp_main.c
        )
