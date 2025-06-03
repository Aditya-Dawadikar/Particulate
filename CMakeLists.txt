cmake_minimum_required(VERSION 3.12)

# Toolchain settings (optional for now since no CUDA yet)
set(CMAKE_C_COMPILER /usr/bin/gcc-10)
set(CMAKE_CXX_COMPILER /usr/bin/g++-10)

project(particulate LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

# Include headers
include_directories(${CMAKE_SOURCE_DIR}/include)

# Find SDL2 if we plan to use it later
find_package(SDL2 REQUIRED)
find_library(SDL2_GFX_LIB SDL2_gfx)
include_directories(${SDL2_INCLUDE_DIRS})

# Collect all source files (CPU only for now)
file(GLOB_RECURSE SRC_FILES src/*.cpp)

# Executable
add_executable(particulate ${SRC_FILES})

# Link SDL2 if renderer.cpp uses it
target_link_libraries(particulate ${SDL2_LIBRARIES} ${SDL2_GFX_LIB})
