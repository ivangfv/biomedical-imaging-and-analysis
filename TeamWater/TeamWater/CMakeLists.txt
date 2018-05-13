# This is the root ITK CMakeLists file.
cmake_minimum_required(VERSION 2.8.12)
if(COMMAND CMAKE_POLICY)
  cmake_policy(SET CMP0003 NEW)
endif()


# This project is designed to be built outside the Insight source tree.
project(TeamWater)

# Find ITK.
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

add_executable(TeamWater TeamWater.cpp DicomSeriesReadImageWrite2.cxx ImageReadImageSeriesWrite.cxx AffineTransform.cxx BSplineRegistration.cxx BSplineRegistration2)

target_link_libraries(TeamWater ${ITK_LIBRARIES})
