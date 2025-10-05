#
# Copyright (c) xEmbeddedTools team and contributors.
# Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
#

cmake_minimum_required(VERSION 3.20.0)

file(GLOB XMCU_SRC "${CMAKE_CURRENT_LIST_DIR}/xmcu/*.cpp"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/*.c"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/*.hpp"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/*.h"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/hal/*.cpp"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/hal/*.c"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/hal/*.hpp"
                   "${CMAKE_CURRENT_LIST_DIR}/xmcu/hal/*.h" 
                   "${CMAKE_CURRENT_LIST_DIR}/soc/st/arm/Systick.cpp" 
                   "${CMAKE_CURRENT_LIST_DIR}/soc/st/arm/Systick.hpp")

set(XMCU_PATH ${CMAKE_CURRENT_LIST_DIR})

if(XMCU_SOC_MODEL STREQUAL XMCU_SOC_MODEL_STM32WB35CEU6A OR XMCU_SOC_MODEL STREQUAL XMCU_SOC_MODEL_STM32WB55CGU6)
    set(XMCU_SOC_PATH "${CMAKE_CURRENT_LIST_DIR}/soc/st/arm/m4/wb/")
elseif(XMCU_SOC_MODEL STREQUAL XMCU_SOC_MODEL_STM32L010F4P6 OR XMCU_SOC_MODEL STREQUAL XMCU_SOC_MODEL_STM32L010C6T6)
    set(XMCU_SOC_PATH "${CMAKE_CURRENT_LIST_DIR}/soc/st/arm/m0/l0/")
else()
    message(FATAL_ERROR "Unsupported SOC model")
endif()

add_library(XMCU OBJECT ${XMCU_SRC})
target_compile_features(XMCU PRIVATE cxx_std_23)
target_compile_features(XMCU PRIVATE c_std_11)

target_include_directories(XMCU PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

add_subdirectory(${XMCU_SOC_PATH} ${CMAKE_BINARY_DIR}/${XMCU_SOC_MODEL})
target_link_libraries(XMCU PRIVATE XMCU_SOC)

message("XMCU: Selected configuration: ${XMCU_SOC_MODEL}")
set(XMCU_SOC_PATH ${XMCU_SOC_PATH} PARENT_SCOPE)