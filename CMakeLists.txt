cmake_minimum_required(VERSION 3.10)
project(space_carving)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)
set(VTK_DIR "/home/himalaya/Documents/VTK-master/build")
#find_package(VTK REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(space_carving
        main.cpp src/extract_contour.cpp src/extract_contour.h src/bg_constraints_carving.cpp src/bg_constraints_carving.h src/show3d.cpp src/show3d.h src/carving.h src/carving.cpp src/pc_constraint_carving.cpp src/pc_constraint_carving.h src/fileio.cpp src/fileio.h)
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${PCL_INCLUDE_DIRS})

target_link_libraries(space_carving ${OpenCV_LIBS})