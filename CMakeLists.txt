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

# Wi-Fi credentials + debug-log destination for wifi_log.c -- put these in a
# .env file next to this one (git-ignored, see .env.example), or set them with
# -DWIFI_SSID=... -DWIFI_PASSWORD=... -DLOG_SERVER_IP=... on the cmake
# command line. Values in .env win over the command line.
set(WIFI_SSID "" CACHE STRING "Wi-Fi SSID for wireless debug logging")
set(WIFI_PASSWORD "" CACHE STRING "Wi-Fi password for wireless debug logging")
set(MQTT_BROKER_IP "" CACHE STRING "MQTT Broker IP for wireless debug logging")

# Read KEY=VALUE lines from .env. Blank lines and # comments are skipped, and
# surrounding quotes are removed. Only the keys above are used. Values can't
# contain ';' (CMake's list separator).
set(DOTENV_FILE ${CMAKE_CURRENT_LIST_DIR}/.env)
if(EXISTS ${DOTENV_FILE})
    # Re-run cmake automatically whenever .env changes.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${DOTENV_FILE})

    file(STRINGS ${DOTENV_FILE} DOTENV_LINES)
    foreach(DOTENV_LINE IN LISTS DOTENV_LINES)
        string(STRIP "${DOTENV_LINE}" DOTENV_LINE)
        if(DOTENV_LINE MATCHES "^([A-Za-z_][A-Za-z0-9_]*)[ \t]*=[ \t]*(.*)$")
            set(DOTENV_KEY ${CMAKE_MATCH_1})
            set(DOTENV_VALUE "${CMAKE_MATCH_2}")
            string(REGEX REPLACE "^\"(.*)\"$" "\\1" DOTENV_VALUE "${DOTENV_VALUE}")
            string(REGEX REPLACE "^'(.*)'$" "\\1" DOTENV_VALUE "${DOTENV_VALUE}")
            if(DOTENV_KEY MATCHES "^(WIFI_SSID|WIFI_PASSWORD|LOG_SERVER_IP|LOG_SERVER_PORT)$")
                # A normal variable hides the cache entry of the same name, so
                # .env is used even if the build folder was configured before.
                set(${DOTENV_KEY} "${DOTENV_VALUE}")
            endif()
        endif()
    endforeach()
endif()

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(RoboPico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(RoboPico
        main.c
        motor.c
        ir_sensor.c
        wifi_log.c
        mqtt_client.c
        )

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(RoboPico 0)
pico_enable_stdio_usb(RoboPico 1)

target_compile_definitions(RoboPico PRIVATE
        WIFI_SSID="${WIFI_SSID}"
        WIFI_PASSWORD="${WIFI_PASSWORD}"
        MQTT_BROKER_IP="${MQTT_BROKER_IP}"
        )

# Add the standard library to the build
target_link_libraries(RoboPico
        pico_stdlib
        hardware_pwm
        hardware_adc
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mqtt
        )

# Add the standard include files to the build
target_include_directories(RoboPico PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(RoboPico)


