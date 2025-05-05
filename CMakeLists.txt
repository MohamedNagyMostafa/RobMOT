cmake_minimum_required(VERSION 3.28)
project(Tracking_3D)
set(CMAKE_C_COMPILER /usr/bin/gcc-13)
set(CMAKE_CXX_COMPILER /usr/bin/g++-13)
set(CMAKE_CXX_STANDARD 17)


find_package(OpenCV 4.1 REQUIRED)
find_package(PCL 1.14 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


add_executable(Tracking_3D
        src/main.cpp
        src/datatype/data_structure.cpp
        src/util.cpp
        src/calibration_module/PointCloudCalibration.cpp
        src/memory_module/MemoryManagement.cpp
        src/association_module/ObjectAssociation.cpp
        src/tracking_module/KF_Tracking.cpp
        src/tracking_module/KalmanFilter.cpp
        src/config/config.cpp
        src/datatype/Box3D.cpp)

target_link_libraries (Tracking_3D ${OpenCV_LIBS})
target_link_libraries (Tracking_3D ${PCL_LIBRARIES})
