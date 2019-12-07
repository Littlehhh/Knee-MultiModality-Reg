cmake_minimum_required(VERSION 3.15)
project(Knee_MultiModality_Reg)

set(CMAKE_CXX_STANDARD 14)

find_package(ITK)
IF ( ITK_FOUND )
    INCLUDE( ${ITK_USE_FILE} )
ENDIF( ITK_FOUND )


add_executable(Knee_MultiModality_Reg main.cpp)