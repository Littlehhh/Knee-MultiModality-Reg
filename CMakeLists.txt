cmake_minimum_required(VERSION 3.10)
project(Knee_MultiModality_Reg)

set(CMAKE_CXX_STANDARD 14)

find_package(ITK REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(VTK REQUIRED)

INCLUDE( ${ITK_USE_FILE} )
INCLUDE( ${VTK_USE_FILE} )
itk_module_config(ITK ${ITK_MODULES_ENABLED})

#message(STATUS "ITK_MODULES_ENABLED: " ${ITK_MODULES_ENABLED})
#message(STATUS "project source: " ${PROJECT_SOURCE_DIR})
#message(STATUS "project binary: " ${PROJECT_BINARY_DIR})
#message(STATUS "ITK libs: " ${ITK_LIBRARIES})
#message(STATUS "ITK USE: " ${ITK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "ITK USE: " ${PCL_VISUALIZATION_LIBRARIES})
add_executable(Knee_MultiModality_Reg main.cpp utils.h itkImageToPointSet.h itkRegistration.cpp)
target_link_libraries(Knee_MultiModality_Reg ${VTK_LIBRARIES} ${ITK_LIBRARIES} ${PCL_LIBRARIES})

