cmake_minimum_required(VERSION 3.0)
project(tsdf_cuda)

find_package(OpenCV REQUIRED)

option(BUILD_CUDA "Build with CUDA support" on)

SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")
endif()

include_directories(${CMAKE_SOURCE_DIR}/include/)
if(BUILD_CUDA)
find_package(CUDA REQUIRED)
SET(SOURCE ${CMAKE_SOURCE_DIR}/src/tsdf.cu
)
SET(HEADERS ${CMAKE_SOURCE_DIR}/include/tsdf.cuh
)
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
SET(CUDA_PROPAGATE_HOST_FLAGS ON)
cuda_add_library(tsdf_cuda SHARED ${SOURCE} ${HEADERS})
endif()
