cmake_minimum_required(VERSION 3.10)
project(Knee_MultiModality_Reg)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.9 REQUIRED)
find_package(EIGEN3 REQUIRED NO_MODULE)
find_package(ITK REQUIRED)
find_package(VTK REQUIRED)
#set(_internal_cmake_eigen3)
#list(GET ITKEigen3_INCLUDE_DIRS 0 _internal_cmake_eigen3)
#set(Eigen3_DIR "${_internal_cmake_eigen3}/itkeigen")
include_directories(${EIGEN_INCLUDE_DIR})

#option(ITK_USE_SYSTEM_EIGEN 1)
INCLUDE( ${ITK_USE_FILE} )
INCLUDE( ${VTK_USE_FILE} )
itk_module_config(ITK ${ITK_MODULES_ENABLED})

#message(STATUS "Eigen3_ROOT: ${Eigen_ROOT}")
#message(STATUS "From ITK: Eigen3_DIR: ${Eigen3_DIR}")
#message(STATUS "ITK_MODULES_ENABLED: " ${ITK_MODULES_ENABLED})
#message(STATUS "project source: " ${PROJECT_SOURCE_DIR})
#message(STATUS "project binary: " ${PROJECT_BINARY_DIR})
#message(STATUS "ITK libs: " ${ITK_LIBRARIES})
#message(STATUS "ITK USE: " ${ITK_USE_SYSTEM_EIGEN})

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Knee_MultiModality_Reg main.cpp utils.h itkImageToPointSet.h itkRegistration.cpp)
target_link_libraries(Knee_MultiModality_Reg Eigen3::Eigen ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${ITK_LIBRARIES} )

