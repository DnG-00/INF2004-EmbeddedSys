// FOR TESTING PURPOSES ONLY, TO REPLACE LATER

#include "wifi_log.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip4_addr.h"

#define WIFI_LOG_LINE_BUF_SIZE 256
#define WIFI_LOG_CONNECT_TIMEOUT_MS 15000
#define WIFI_LOG_CONNECT_RETRIES 3

static struct udp_pcb *s_udp_pcb;
static ip_addr_t s_server_addr;
static uint16_t s_server_port;

static char s_line_buf[WIFI_LOG_LINE_BUF_SIZE];
static size_t s_line_len;
static volatile uint32_t s_dropped_count;

static void wifi_log_send_line(void) {
    if (s_line_len == 0 || !s_udp_pcb) return;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)s_line_len, PBUF_RAM);
    if (p) {
        memcpy(p->payload, s_line_buf, s_line_len);
        cyw43_arch_lwip_begin();
        udp_sendto(s_udp_pcb, p, &s_server_addr, s_server_port);
        cyw43_arch_lwip_end();
        pbuf_free(p);
    } else {
        s_dropped_count++;
    }
    s_line_len = 0;
}

// stdio_driver_t out_chars callback -- called for every printf/putchar,
// alongside whatever other drivers (USB, UART) are already registered.
// Buffers characters until a newline (or the buffer fills), then ships the
// line as one UDP packet.
static void wifi_log_out_chars(const char *buf, int len) {
    for (int i = 0; i < len; i++) {
        char c = buf[i];
        if (c == '\r') continue;
        if (c == '\n') {
            wifi_log_send_line();
            continue;
        }
        if (s_line_len >= sizeof(s_line_buf) - 1) {
            wifi_log_send_line();
        }
        s_line_buf[s_line_len++] = c;
    }
}

static stdio_driver_t s_wifi_stdio_driver = {
    .out_chars = wifi_log_out_chars,
};

bool wifi_log_init(const char *ssid, const char *password,
                    const char *server_ip, uint16_t server_port) {
    if (cyw43_arch_init()) {
        return false;
    }
    cyw43_arch_enable_sta_mode();

    // A single failed attempt is common right after a reset: a reset doesn't
    // send a clean 802.11 deauth, so some routers are slow to let the same
    // client reassociate until they time out the stale session. Retrying
    // absorbs that instead of requiring a manual power-cycle-and-hope.
    int result = -1;
    for (int attempt = 1; attempt <= WIFI_LOG_CONNECT_RETRIES; attempt++) {
        result = cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK,
                                                      WIFI_LOG_CONNECT_TIMEOUT_MS);
        if (result == 0) break;
        printf("Wi-Fi connect attempt %d/%d failed, retrying...\n", attempt, WIFI_LOG_CONNECT_RETRIES);
    }
    if (result != 0) {
        return false;
    }

    if (!ip4addr_aton(server_ip, &s_server_addr)) {
        return false;
    }
    s_server_port = server_port;

    cyw43_arch_lwip_begin();
    s_udp_pcb = udp_new();
    cyw43_arch_lwip_end();
    if (!s_udp_pcb) {
        return false;
    }

    stdio_set_driver_enabled(&s_wifi_stdio_driver, true);
    return true;
}

uint32_t wifi_log_get_dropped_count(void) {
    return s_dropped_count;
}
