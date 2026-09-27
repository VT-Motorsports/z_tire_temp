cmake_minimum_required(VERSION 3.20.0)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(TTS_tests)



target_sources(app PRIVATE
    # App files
    ../../src/thermal_pipeline.cpp
    ../../include/thermal_pipeline.h

    # Test files
    src/main.cpp 
    ../common/test_access.hpp
)



