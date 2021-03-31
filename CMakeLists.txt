cmake_minimum_required(VERSION 2.8)
project(orbslam2Map)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING -Wno-ignored-attributes -Wno-unused-result")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING -Wno-ignored-attributes -Wno-unused-result")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

find_package(OpenCV 4.5.1 QUIET)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 4.5.1 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS} /usr/local/include/ORB_SLAM2)

set(LIBS -lfbow -lDLib -lg2o -lORB_SLAM2 -lboost_system)

find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
  message(FATAL_ERROR "Can't find libboost_serialization")
endif()

# Build Sources to executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)
add_executable(loadMap src/loadMap.cc)
target_link_libraries(loadMap ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES} ${LIBS} ${BOOST_SERIALIZATION})