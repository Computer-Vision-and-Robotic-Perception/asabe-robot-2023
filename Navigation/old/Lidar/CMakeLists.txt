cmake_minimum_required(VERSION 3.22)
project(Lidar)

set(CMAKE_CXX_STANDARD 14)

add_executable(Lidar main.cpp rplidar_driver_impl.h rplidar_cmd.h rplidar_protocol.h rptypes.h Monte_Carlo_Localization.h)
