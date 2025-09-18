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

if(XMCU_SOC_MODEL STREQUAL XMCU_SOC_MODEL_STM32WB35CEU6A OR
   XMCU_SOC_MODEL STREQUAL XMCU_SOC_MODEL_STM32WB55CGU6)
   set(XMCU_SOC_CMAKE_FILE "${CMAKE_CURRENT_LIST_DIR}/soc/st/arm/m4/wb/CMakeLists.txt")
elseif(XMCU_SOC_MODEL STREQUAL XMCU_SOC_MODEL_STM32L010F4P6 OR
    XMCU_SOC_MODE STREQUAL XMCU_SOC_MODEL_STM32L0110C6T6)
   set(XMCU_SOC_CMAKE_FILE "${CMAKE_CURRENT_LIST_DIR}/soc/st/arm/m0/l0/CMakeLists.txt")
else()
    message(FATAL_ERROR "Unsupported SOC model")
endif()

file(GLOB XMCU_SRC ${XMCU_SRC} "${CMAKE_CURRENT_LIST_DIR}/soc/st/arm/Systick.cpp" "${CMAKE_CURRENT_LIST_DIR}/soc/st/arm/Systick.hpp")

add_library(XMCU OBJECT ${XMCU_SRC})
include_directories("${CMAKE_CURRENT_LIST_DIR}")

set(XMCU_PATH ${CMAKE_CURRENT_LIST_DIR})

message("XMCU_SOC_CMAKE_FILE: ${XMCU_SOC_CMAKE_FILE}")
if(EXISTS ${XMCU_SOC_CMAKE_FILE})
    include(${XMCU_SOC_CMAKE_FILE})
else()
    message(FATAL_ERROR "No submodule found!")
endif()

message("XMCU: Selected configuration: ${XMCU_SOC_MODEL}")

