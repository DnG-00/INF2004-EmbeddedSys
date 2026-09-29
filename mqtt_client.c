#include "mqtt_client.h"

#include <stdio.h>
#include <string.h>

#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"

static mqtt_client_t *mqtt_client = NULL;
static ip_addr_t broker_address;
static bool mqtt_connected = false;

static void mqtt_connection_callback(
    mqtt_client_t *client,
    void *arg,
    mqtt_connection_status_t status)
{
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT connected!\n");
        mqtt_connected = true;
    } else {
        printf("MQTT connection failed: %d\n", status);
        mqtt_connected = false;
    }
}

bool mqtt_init(const char *broker_ip)
{
    if (!ip4addr_aton(broker_ip, &broker_address)) {
        printf("Invalid MQTT broker IP\n");
        return false;
    }

    mqtt_client = mqtt_client_new();

    if (!mqtt_client) {
        printf("Failed to create MQTT client\n");
        return false;
    }

    struct mqtt_connect_client_info_t client_info = {
        .client_id = "pico_w_car",
        .client_user = NULL,
        .client_pass = NULL,
        .keep_alive = 60,
        .will_topic = NULL,
        .will_msg = NULL,
        .will_qos = 0,
        .will_retain = 0
    };

    cyw43_arch_lwip_begin();

    err_t err = mqtt_client_connect(
        mqtt_client,
        &broker_address,
        1883,
        mqtt_connection_callback,
        NULL,
        &client_info
    );

    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("MQTT connection error: %d\n", err);
        return false;
    }

    printf("MQTT connection started...\n");

    return true;
}

bool mqtt_publish_test(void)
{
    if (!mqtt_client || !mqtt_connected) {
        printf("MQTT is not connected yet\n");
        return false;
    }

    const char *message = "Hello MQTT from Pico W";

    cyw43_arch_lwip_begin();

    err_t err = mqtt_publish(
        mqtt_client,
        "car/test",
        message,
        strlen(message),
        0,
        0,
        NULL,
        NULL
    );

    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("MQTT publish failed: %d\n", err);
        return false;
    }

    printf("MQTT message published!\n");

    return true;
}