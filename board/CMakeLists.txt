cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 用户代码（bsp / libs / modules / threads）全部是 C++，所以要把 C++17 打开
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME board)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ---------------------------------------------------------------------------
# 用户代码
#   board/ 是 CubeMX 生成的板级工程（本文件就是它的构建入口），
#   用户代码在仓库根目录的同级目录里：
#       bsp/     外设封装       can / usart / pwm
#       libs/    与硬件无关的算法 pid / math / crc
#       modules/ 设备驱动       BMI088 / M2006 / DJIMotorHandler
#       threads/ ThreadX 线程   alive / imu / control
#   依赖方向单向：threads → modules → libs/bsp，下层不认识上层。
# ---------------------------------------------------------------------------
set(USER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # bsp：外设封装
    ${USER_ROOT}/bsp/src/bsp.cpp
    ${USER_ROOT}/bsp/src/bsp_can.cpp
    ${USER_ROOT}/bsp/src/bsp_pwm.cpp
    ${USER_ROOT}/bsp/src/bsp_usart.cpp

    # libs：算法
    ${USER_ROOT}/libs/src/pid.cpp
    ${USER_ROOT}/libs/src/math.cpp
    ${USER_ROOT}/libs/src/crc.cpp

    # modules：设备
    ${USER_ROOT}/modules/src/BMI088.cpp
    ${USER_ROOT}/modules/src/DJIMotorHandler.cpp
    ${USER_ROOT}/modules/src/M2006.cpp

    # threads：ThreadX 线程
    ${USER_ROOT}/threads/src/alivethread.cpp
    ${USER_ROOT}/threads/src/imuthread.cpp
    ${USER_ROOT}/threads/src/controlthread.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_ROOT}/bsp/include
    ${USER_ROOT}/libs/include
    ${USER_ROOT}/modules/include
    ${USER_ROOT}/threads/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
