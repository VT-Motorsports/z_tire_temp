# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

#generates compile_commands.json for clangd to LSP correctly. 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(vcu)

# Add include/ directory to include paths
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Gather all C and C++ source files under src/
file(GLOB_RECURSE APP_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# Suppress conversion/promotion warnings for vendor MLX90640 library
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MLX90640_API.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MLX90640_I2C_Driver.c
    PROPERTIES COMPILE_FLAGS "-Wno-double-promotion -Wno-conversion"
)

# Add sources to app target
target_sources(app PRIVATE ${APP_SOURCES})


