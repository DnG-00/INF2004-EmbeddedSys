#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h" // C's library that supports the I/O devices
#include "hardware/gpio.h" // C's library that supports GPIO connection
#include "pico/cyw43_arch.h" // C's library that supports PICO W's in-built WIFI
#include "lwipopts.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "pico/error.h"
#include "mqtt_topics.h" // Header file that contains topics where devices can publish & subscribe
#include "mqtt_client.h"

bool is_wifi_connected(int value) { // cyw43_arch_wifi_connect_timeout_ms (const char *ssid, const char *pw, uint32_t auth, uint32_t timeout) display errors by representing them using 0 to -21
    if (value < 0) {
        return true;
    } else {
        return false;
    }
};

bool wifi_init(void) {
    // To initialize the WIFI chip
    if (cyw43_arch_init()) { // To check if the WIFI chip failed to initialize
        printf("WIFI chip init failed\n");
        return false;
    } else {
        cyw43_arch_enable_sta_mode(); // To enable the WIFI station mode (a state where a wireless device connects to an existing WIFI network)
    
        printf("Connecting to %s...\n", WIFI_SSID); // The PICO W tries to connect with the WIFI

        int wifi_connect_error_code = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000); // To check if there is an error connecting to the WIFI connection
        
        if (is_wifi_connected(wifi_connect_error_code)) {
             switch (wifi_connect_error_code)
            {
            case PICO_ERROR_GENERIC:
                printf("WIFI connection has an unspecified error occured!\n");
                break;

            case PICO_ERROR_TIMEOUT:
                printf("WIFI connection timed out!\n");
                break;

            case PICO_ERROR_BADAUTH:
                printf("Authentication failed!\n");
                break;

            case PICO_ERROR_CONNECT_FAILED:
                printf("WIFI connection has failed to connect!\n");
                break;

            default:
                printf("WIFI connection failed! [ERROR: %d]\n", wifi_connect_error_code);
                break;
            }
        } else {
            printf("WIFI connection successfully!\n");
            return true;
        }
    }
}

/*

TODO:
- Connect the MQTT broker to the PICO
- Deal with formatting of payload
- Deal with the flow of publishing and subcribing with the topics
- Deal with the WIFI disconnection (use USB in the meantime)
- Deal with MQTT disconnection
- Deal with WIFI reconnection (polling)
- Deal with MQTT reconnection (polling)
- Deal with heartbeat checks for WIFI and MQTT
- Retrieve all the data from all the sensors
- Encode the data from the sensors
- Print the data from the sensors

*/ 

