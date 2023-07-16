#############################################################
# Copyright (c) 2022, University of California, Los Angeles #
#                                                           #
# Modifier: Eungchang Mason Lee                             #
# Authors: Kenny J. Chen, Brett T. Lopez                    #
# Contact: kennyjchen@ucla.edu, btlopez@ucla.edu            #
#############################################################

cmake_minimum_required(VERSION 3.10.0)
project(nano_gicp)

### set compiler
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -fexceptions -g -ggdb -fopenmp") #openmp here
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)

### get packages
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin)
catkin_package( #this automatically adds/installs the results of build
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

### get packages' headers
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

###########
## Build ##
###########
### NanoFLANN
add_library(nanoflann STATIC src/nanoflann.cc)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
### NanoGICP
add_library(nano_gicp STATIC src/nano_gicp.cc)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} nanoflann)