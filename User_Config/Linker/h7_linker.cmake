# Project-owned linker selection. Included from the CubeMX-preserved top-level CMakeLists.
# Generated GCC and STARM toolchains add their default -T option; replace that option
# here so regeneration cannot switch the application back to the generated memory map.
set(H7_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/h7_memory.ld")
set(_h7_generated_linker_option "-T \"${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld\"")
string(FIND "${CMAKE_EXE_LINKER_FLAGS}" "${_h7_generated_linker_option}"
    _h7_generated_linker_index)
if(_h7_generated_linker_index EQUAL -1)
    message(FATAL_ERROR
        "The CubeMX linker option changed. Review User_Config/Linker/h7_linker.cmake "
        "before building; the generated memory map must not be used.")
endif()
string(REPLACE "${_h7_generated_linker_option}" ""
    CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
unset(_h7_generated_linker_option)
unset(_h7_generated_linker_index)

set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY
    LINK_DEPENDS "${H7_LINKER_SCRIPT}")
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "-T${H7_LINKER_SCRIPT}")
message(STATUS "H7_BSP linker script: ${H7_LINKER_SCRIPT}")
