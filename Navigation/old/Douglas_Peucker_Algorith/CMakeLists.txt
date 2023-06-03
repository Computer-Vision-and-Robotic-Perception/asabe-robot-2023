cmake_minimum_required(VERSION 3.22)
project(Douglas_Peucker_Algorithm)

set(CMAKE_CXX_STANDARD 14)

add_executable(Douglas_Peucker_Algorithm main.cpp Douglas_Peucker.cpp Douglas_Peucker.h)
