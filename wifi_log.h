// FOR TESTING PURPOSES ONLY, TO REPLACE LATER

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Connects to Wi-Fi and mirrors all stdio output (printf, puts, etc.) to
// UDP packets sent to server_ip:server_port, IN ADDITION to whatever stdio
// drivers are already active (e.g. USB serial) -- existing printf() calls
// don't need to change. Call once, after stdio_init_all().
//
// Run wifi_log_listener.py on server_ip to see the output without a USB
// cable connected.
//
// Retries the connection a few times before giving up (a single failed
// attempt right after a reset is common -- see wifi_log.c), so this can
// block for up to ~3x the per-attempt timeout in the worst case. Returns
// true if the Wi-Fi connection succeeded. On failure, USB/other stdio
// output still works as normal.
bool wifi_log_init(const char *ssid, const char *password,
                    const char *server_ip, uint16_t server_port);

// Number of lines that couldn't be sent (pbuf allocation failed, usually
// under bursty printing) since boot. Non-zero means lines were silently
// lost -- if this climbs, MEM_SIZE in lwipopts.h needs to go up further.
uint32_t wifi_log_get_dropped_count(void);
