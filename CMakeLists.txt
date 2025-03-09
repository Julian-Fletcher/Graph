cmake_minimum_required(VERSION 3.10)

# Set compiler to g++-13
find_program(GXX13 g++-13) # 13 to support c++ 23

if(GXX13)
  set(CMAKE_CXX_COMPILER ${GXX13})
  message(STATUS "Using g++ version: ${GXX13}")
else()
  message(FATAL_ERROR "g++-13 not found!")
endif()

# Set to C++23
set(CMAKE_CXX_STANDARD 23)

project(Planar)

# Add executable (src files)
add_executable(planar 
					src/planar.cpp 
					)
# Specify include directories (h files)
target_include_directories(planar PUBLIC include/)