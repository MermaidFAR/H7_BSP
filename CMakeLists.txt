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

set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")

set(H7_FREERTOS_DTCM_HEAP_SIZE "49152" CACHE STRING
    "FreeRTOS heap bytes retained in DTCMRAM; the remainder is placed in RAM_D1")
if(NOT H7_FREERTOS_DTCM_HEAP_SIZE MATCHES "^[0-9]+$")
    message(FATAL_ERROR "H7_FREERTOS_DTCM_HEAP_SIZE must be an unsigned integer")
endif()
if(H7_FREERTOS_DTCM_HEAP_SIZE LESS 1024 OR
   H7_FREERTOS_DTCM_HEAP_SIZE GREATER_EQUAL 65536)
    message(FATAL_ERROR
        "H7_FREERTOS_DTCM_HEAP_SIZE must be in [1024, 65535]")
endif()
math(EXPR H7_FREERTOS_DTCM_HEAP_REMAINDER
    "${H7_FREERTOS_DTCM_HEAP_SIZE} % 8")
if(NOT H7_FREERTOS_DTCM_HEAP_REMAINDER EQUAL 0)
    message(FATAL_ERROR
        "H7_FREERTOS_DTCM_HEAP_SIZE must be 8-byte aligned")
endif()
message(STATUS
    "FreeRTOS heap: ${H7_FREERTOS_DTCM_HEAP_SIZE} bytes in DTCMRAM, remainder in RAM_D1")

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
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY
    LINK_DEPENDS
        "${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld"
        "${CMAKE_SOURCE_DIR}/User_Config/Linker/h7_memory_sections.ld"
)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-T${CMAKE_SOURCE_DIR}/User_Config/Linker/h7_memory_sections.ld"
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

get_target_property(RTOS_SRC FreeRTOS SOURCES)

list(FILTER RTOS_SRC EXCLUDE REGEX "port\\.c$")

list(APPEND RTOS_SRC
    "${CMAKE_SOURCE_DIR}/User_Config/FreeRTOS_Patch/port_patched.c"
    "${CMAKE_SOURCE_DIR}/User_Config/FreeRTOS_Patch/heap_regions_patched.c"
)

set_target_properties(FreeRTOS PROPERTIES SOURCES "${RTOS_SRC}")
target_compile_definitions(FreeRTOS PRIVATE
    configHEAP_5_REGIONS=SYS_FreeRTOS_Heap_Regions
    H7_FREERTOS_DTCM_HEAP_SIZE=${H7_FREERTOS_DTCM_HEAP_SIZE}U
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
    Middlewares/ST/ARM/DSP/Lib
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Basic/alg_basic.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Filter/EKF/alg_filter_ekf.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/FSM/alg_fsm.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Filter/Frequency/alg_filter_frequency.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Filter/IIR/alg_filter_iir.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Filter/Kalman/alg_filter_kalman.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Matrix/alg_matrix.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/PID/alg_pid.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Complex/alg_complex.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Quaternion/alg_quaternion.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Queue/alg_queue.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Slope/alg_slope.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/BSP/ADC/bsp_adc.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/BSP/SPI/bsp_spi.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/callback/callback.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/Timestamp/sys_timestamp.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/Power/bsp_power.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/BMI088/bsp_bmi088.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/BMI088/Accel/bsp_bmi088_accel.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/BMI088/Accel/bsp_bmi088_accel_register.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/BMI088/Gyro/bsp_bmi088_gyro.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/BMI088/Gyro/bsp_bmi088_gyro_register.cpp
        ${CMAKE_CURRENT_LIST_DIR}/SystemView/SEGGER/SEGGER_RTT.c
        ${CMAKE_CURRENT_LIST_DIR}/SystemView/SEGGER/SEGGER_SYSVIEW.c
        ${CMAKE_CURRENT_LIST_DIR}/SystemView/SEGGER/SEGGER_RTT_printf.c
        ${CMAKE_CURRENT_LIST_DIR}/SystemView/FreeRTOS/SEGGER_SYSVIEW_FreeRTOS.c
        ${CMAKE_CURRENT_LIST_DIR}/SystemView/FreeRTOS/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c
        ${CMAKE_CURRENT_LIST_DIR}/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.S
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/Init/Init.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Task/InsTask.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Task/TransportTask.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Task/CanTxTask.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/debug/sys_debug.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/Storage/sys_imu_bias_store.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/BSP/CAN/bsp_can.c
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/BSP/UART/bsp_uart.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/BSP/USB/bsp_usb.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/BSP/OSPI/bsp_ospi.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/W25Q64JV/bsp_w25q64jv_register.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/W25Q64JV/bsp_w25q64jv.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/Buzzer/bsp_buzzer.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/Key/bsp_key.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/WS2812/bsp_ws2812.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Peripheral/EricTool/dvc_erictool.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Task/TIM_1ms_Task.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Task/BMI088_Task.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Task/StorageTask.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Peripheral/Motor/QDrive/QD4310.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Pulse/alg_pulse.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Task/Control_Task.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Application/Gimbal/Gimbal.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/User_File/Application/Communication/Communication.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Application/Balance/balance.cpp
        User_File/Application/Communication/Com.cpp

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        User_File/Middleware/Algorithm/Basic
        User_File/Middleware/Algorithm/Complex
        User_File/Middleware/Algorithm/Filter/EKF
        User_File/Middleware/Algorithm/Filter/Frequency
        User_File/Middleware/Algorithm/Filter/IIR
        User_File/Middleware/Algorithm/Filter/Kalman
        User_File/Middleware/Algorithm/FSM
        User_File/Middleware/Algorithm/Matrix
        User_File/Middleware/Algorithm/PID
        User_File/Middleware/Algorithm/Quaternion
        User_File/Middleware/Algorithm/Queue
        User_File/Middleware/Algorithm/Slope
        User_File/Middleware/BSP/ADC
        User_File/Middleware/BSP/SPI
        User_File/System/callback
        User_File/System/Timestamp
        User_File/Device/Onboard/Power
        User_File/Device/Onboard/BMI088
        User_File/Device/Onboard/BMI088/Accel
        User_File/Device/Onboard/BMI088/Gyro
        Middlewares/ST/ARM/DSP/Inc
        User_File/System/Init
        User_File/Task
        User_File/System/debug
        User_File/System/Storage
        User_File/Middleware/BSP/CAN
        User_File/Middleware/BSP/UART
        User_File/Middleware/BSP/USB
        User_File/Middleware/BSP/OSPI
        User_File/Device/Onboard/W25Q64JV
        User_File/Device/Onboard/Buzzer
        User_File/Device/Onboard/Key
        User_File/Device/Onboard/WS2812
        User_File/Device/Peripheral/EricTool
        User_File/Device/Peripheral/Motor/QDrive
        User_File/Middleware/Algorithm/Pulse/
        User_File/Application/Gimbal/
        User_File/Application/Communication/
        User_File/Application/Balance/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    BALANCE=1
    GIMBAL=0
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    arm_cortexM7lfsp_math
)
