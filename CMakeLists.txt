cmake_minimum_required(VERSION 3.10)
project(Knee_MultiModality_Reg)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.9 REQUIRED)
find_package(EIGEN3 REQUIRED NO_MODULE)
find_package(ITK REQUIRED)
find_package(VTK REQUIRED)


include_directories(${EIGEN_INCLUDE_DIR})


INCLUDE( ${ITK_USE_FILE} )
itk_module_config(ITK ${ITK_MODULES_ENABLED})

INCLUDE( ${VTK_USE_FILE} )



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Knee_MultiModality_Reg main.cpp utils.h itkImageToPointSet.h itkRegistration.cpp itkImageTovtkPolyData.h itkImageTopclPointCloud.h)
target_link_libraries(Knee_MultiModality_Reg Eigen3::Eigen ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${ITK_LIBRARIES} )

