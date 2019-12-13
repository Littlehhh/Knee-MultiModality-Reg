cmake_minimum_required(VERSION 3.15)
project(Knee_MultiModality_Reg)

set(CMAKE_CXX_STANDARD 14)

find_package(ITK)
INCLUDE( ${ITK_USE_FILE} )

add_executable(Knee_MultiModality_Reg main.cpp)
target_link_libraries(Knee_MultiModality_Reg ${ITK_LIBRARIES})