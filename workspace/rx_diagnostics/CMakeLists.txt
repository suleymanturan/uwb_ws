# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.13.1)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/spm.conf")
  set(spm_CONF_FILE
    prj.conf
    ${CMAKE_CURRENT_LIST_DIR}/spm.conf
  )
endif()
 
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mcuboot.conf")
  set(mcuboot_CONF_FILE
    prj.conf
    ${CMAKE_CURRENT_LIST_DIR}/mcuboot.conf
  )
endif()
 
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${BOARD}.overlay")
  set(mcuboot_DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${BOARD}.overlay")
  set(spm_DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${BOARD}.overlay")
endif()

set(BOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(DTS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(BOARD qorvo_dwm3001cdk)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rx_diagnostics)

# the binary only library from Qorvo has problems with -Os or -O2
zephyr_library_compile_options(-O1)

target_sources(app PRIVATE
  src/main.c
)

target_include_directories(app PRIVATE ../../drivers/platform)

