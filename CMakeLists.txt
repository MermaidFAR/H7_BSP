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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME H7_BSP)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

include_directories(
    ${CMAKE_SOURCE_DIR}/SystemView/SEGGER
    ${CMAKE_SOURCE_DIR}/SystemView/FreeRTOS
    ${CMAKE_SOURCE_DIR}/SystemView/Config
    ${CMAKE_SOURCE_DIR}/SystemView/FreeRTOS/Config

)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

get_target_property(RTOS_SRC FreeRTOS SOURCES)

list(FILTER RTOS_SRC EXCLUDE REGEX "port\\.c$")

list(APPEND RTOS_SRC "${CMAKE_SOURCE_DIR}/User_Config/FreeRTOS_Patch/port_patched.c")

set_target_properties(FreeRTOS PROPERTIES SOURCES "${RTOS_SRC}")

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        User_File/Middleware/Algorithm/Basic/alg_basic.cpp
        User_File/Middleware/Algorithm/Filter/EKF/alg_filter_ekf.cpp
        User_File/Middleware/Algorithm/FSM/alg_fsm.cpp
        User_File/Middleware/Algorithm/Filter/Frequency/alg_filter_frequency.cpp
        User_File/Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.cpp
        User_File/Middleware/Algorithm/Matrix/alg_matrix.cpp
        User_File/Middleware/Algorithm/PID/alg_pid.cpp
        User_File/Middleware/Algorithm/Complex/alg_complex.cpp
        User_File/Middleware/Algorithm/Quaternion/alg_quaternion.cpp
        User_File/Middleware/Algorithm/Queue/alg_queue.cpp
        User_File/Middleware/Algorithm/Slope/alg_slope.cpp
        User_File/Middleware/BSP/ADC/bsp_adc.cpp
        User_File/Middleware/BSP/SPI/bsp_spi.cpp
        User_File/Middleware/System/callback/tim_callback.cpp
        User_File/Middleware/System/Timestamp/sys_timestamp.cpp
        User_File/Device/Components/Power/bsp_power.cpp
        User_File/Device/Components/BMI088/bsp_bmi088.cpp
        User_File/Device/Components/BMI088/Accel/bsp_bmi088_accel.cpp
        User_File/Device/Components/BMI088/Accel/bsp_bmi088_accel_register.cpp
        User_File/Device/Components/BMI088/Gyro/bsp_bmi088_gyro.cpp
        User_File/Device/Components/BMI088/Gyro/bsp_bmi088_gyro_register.cpp
        SystemView/SEGGER/SEGGER_RTT.c
        SystemView/SEGGER/SEGGER_SYSVIEW.c
        SystemView/SEGGER/SEGGER_RTT_printf.c
        SystemView/FreeRTOS/SEGGER_SYSVIEW_FreeRTOS.c
        SystemView/FreeRTOS/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c
        SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.S
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        User_File/Middleware/Algorithm/Basic
        User_File/Middleware/Algorithm/Complex
        User_File/Middleware/Algorithm/Filter/EKF
        User_File/Middleware/Algorithm/Filter/Frequency
        User_File/Middleware/Algorithm/Filter/Kalman
        User_File/Middleware/Algorithm/FSM
        User_File/Middleware/Algorithm/Matrix
        User_File/Middleware/Algorithm/PID
        User_File/Middleware/Algorithm/Quaternion
        User_File/Middleware/Algorithm/Queue
        User_File/Middleware/Algorithm/Slope
        User_File/Middleware/BSP/ADC
        User_File/Middleware/BSP/SPI
        User_File/Middleware/System/callback
        User_File/Middleware/System/Timestamp
        User_File/Device/Components/Power
        User_File/Device/Components/BMI088
        User_File/Device/Components/BMI088/Accel
        User_File/Device/Components/BMI088/Gyro
        Middlewares/ST/ARM/DSP/Inc
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
