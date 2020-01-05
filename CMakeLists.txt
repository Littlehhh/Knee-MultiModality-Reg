cmake_minimum_required(VERSION 3.10)
project(Knee_MultiModality_Reg)

set(CMAKE_CXX_STANDARD 14)

find_package(ITK REQUIRED)
INCLUDE( ${ITK_USE_FILE} )
itk_module_config(ITK ${ITK_MODULES_ENABLED})
message(STATUS "project source: " ${PROJECT_SOURCE_DIR})
message(STATUS "project binary: " ${PROJECT_BINARY_DIR})
message(STATUS "ITK libs: " ${ITK_LIBRARIES})
message(STATUS "ITK USE: " ${ITK_USE_FILE})
add_executable(Knee_MultiModality_Reg main.cpp utils.h)
target_link_libraries(Knee_MultiModality_Reg ${ITK_LIBRARIES})