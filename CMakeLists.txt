cmake_minimum_required(VERSION 3.13)
project(vpcr VERSION 1.0 LANGUAGES C CXX DESCRIPTION "Vulkan Point Cloud Renderer")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()


if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
endif(CMAKE_COMPILER_IS_GNUCXX)
if(WIN32)
    add_definitions(-DNOMINMAX)
endif(WIN32)

find_package(Threads REQUIRED)

add_subdirectory(external)
add_subdirectory(shaders)
add_subdirectory(src)
