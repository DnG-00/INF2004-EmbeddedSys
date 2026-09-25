# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Wi-Fi credentials + debug-log destination for wifi_log.c -- set these with
# -DWIFI_SSID=... -DWIFI_PASSWORD=... -DLOG_SERVER_IP=... on the cmake
# command line (or via the CMake Tools UI) rather than editing source.
set(WIFI_SSID "" CACHE STRING "Wi-Fi SSID for wireless debug logging")
set(WIFI_PASSWORD "" CACHE STRING "Wi-Fi password for wireless debug logging")
set(LOG_SERVER_IP "" CACHE STRING "IP address of the PC running wifi_log_listener.py")
set(LOG_SERVER_PORT "4210" CACHE STRING "UDP port wifi_log_listener.py listens on")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(RoboPico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(RoboPico
        main.c
        motor_controller.c
        wifi_log.c
        )


# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(RoboPico 0)
pico_enable_stdio_usb(RoboPico 1)

target_compile_definitions(RoboPico PRIVATE
        WIFI_SSID="${WIFI_SSID}"
        WIFI_PASSWORD="${WIFI_PASSWORD}"
        LOG_SERVER_IP="${LOG_SERVER_IP}"
        LOG_SERVER_PORT=${LOG_SERVER_PORT}
        )

# Add the standard library to the build
target_link_libraries(RoboPico
        pico_stdlib
        hardware_pwm
        pico_cyw43_arch_lwip_threadsafe_background
        )

# Add the standard include files to the build
target_include_directories(RoboPico PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(RoboPico)

# CMakeList.txt for testing the encoder and motor controller code.
# add_executable(EncoderTest
#         encoder_test.c
#         motor_controller.c
#         )

# pico_set_program_name(EncoderTest "EncoderTest")
# pico_set_program_version(EncoderTest "0.1")

# pico_enable_stdio_uart(EncoderTest 0)
# pico_enable_stdio_usb(EncoderTest 1)

# target_link_libraries(EncoderTest
#         pico_stdlib
#         hardware_pwm
#         )

# target_include_directories(EncoderTest PRIVATE
#         ${CMAKE_CURRENT_LIST_DIR}
# )

# pico_add_extra_outputs(EncoderTest)

