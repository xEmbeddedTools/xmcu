#
# Copyright (c) xEmbeddedTools team and contributors.
# Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
#

cmake_minimum_required(VERSION 3.19.0)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD 11)

file(GLOB XMCU_SRC "${CMAKE_CURRENT_LIST_DIR}/xmcu/*.cpp" 
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/*.c"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/*.hpp"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/*.h"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/hal/*.cpp" 
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/hal/*.c"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/hal/*.hpp"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/hal/*.h")

if (XMCU_SOC_MODEL STREQUAL XMCU_SOC_MODEL_STM32WB35CEU6A)

    file(GLOB XMCU_SRC ${XMCU_SRC} "${CMAKE_CURRENT_LIST_DIR}/soc/ST/arm/Systick.cpp")

    add_library(XMCU OBJECT ${XMCU_SRC})
    include_directories(XMCU PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

    set(XMCU_PATH ${CMAKE_CURRENT_LIST_DIR})

    set(XMCU_SOC_CMAKE_FILE "${CMAKE_CURRENT_LIST_DIR}/soc/ST/arm/m4/wb/CMakeLists.txt")

    if(EXISTS ${XMCU_SOC_CMAKE_FILE})
        include(${XMCU_SOC_CMAKE_FILE})
    else()
        message("No submodule found!")
    endif()
elseif(XMCU_SOC_MODEL STREQUAL XMCU_SOC_MODEL_STM32L010F4P6)

    file(GLOB XMCU_SRC ${XMCU_SRC} "${CMAKE_CURRENT_LIST_DIR}/soc/ST/arm/Systick.cpp")

    add_library(XMCU OBJECT ${XMCU_SRC})
    include_directories(XMCU PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

    set(XMCU_PATH ${CMAKE_CURRENT_LIST_DIR})

    set(XMCU_SOC_CMAKE_FILE "${CMAKE_CURRENT_LIST_DIR}/soc/ST/arm/m0/l0/CMakeLists.txt")

    if(EXISTS ${XMCU_SOC_CMAKE_FILE})
        include(${XMCU_SOC_CMAKE_FILE})
    else()
        message("No submodule found!")
    endif()
else()
    message("Unsupported SOC model")
endif()

message("XMCU: Selected configuration: ${XMCU_SOC_MODEL}")