cmake_minimum_required(VERSION 3.19.0)
project(xmcu LANGUAGES ASM C CXX)
set(CMAKE_CXX_STANDARD 23)

file(GLOB_RECURSE XMCU_SRC "${CMAKE_CURRENT_LIST_DIR}/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/*.c" "${CMAKE_CURRENT_LIST_DIR}/*.asm")
add_library(XMCU ${XMCU_SRC})

if (XMCU_SOC_MODEL STREQUAL "XMCU_SOC_MODEL_STM32WB35CEU6A")
    add_compile_definitions("XMCU_SOC_MODEL_STM32WB35CEU6A")
    set(XMCU_SOC_CMAKE_FILE "${CMAKE_CURRENT_LIST_DIR}/soc/ST/arm/m4/wb/rm0434/CMakeLists.txt")
    set(XMCU_PATH ${CMAKE_CURRENT_LIST_DIR})

    if(EXISTS "${XMCU_SOC_CMAKE_FILE}")
        include("${XMCU_SOC_CMAKE_FILE}")
    endif()
else()
    message(FATAL "Unsupported SOC model")
endif()