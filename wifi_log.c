/** @file wifi_log.c
 *
 * @brief Mirrors printf() output over Wi-Fi as UDP packets.
 *
 * Each printed line is sent as one UDP packet to a PC running
 * wifi_log_listener.py, IN ADDITION to the other stdio outputs (e.g. USB
 * serial), so existing printf() calls don't need to change.
 *
 * FOR TESTING PURPOSES ONLY, TO REPLACE LATER.
 */

#include "wifi_log.h"

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip4_addr.h"

#define WIFI_LOG_LINE_BUF_SIZE      256U
#define WIFI_LOG_CONNECT_TIMEOUT_MS 15000U
#define WIFI_LOG_CONNECT_RETRIES    3

static struct udp_pcb *   g_p_udp_pcb = NULL;
static ip_addr_t          g_server_addr;
static uint16_t           g_server_port = 0U;

static char               g_line_buf[WIFI_LOG_LINE_BUF_SIZE];
static size_t             g_line_len = 0U;
static volatile uint32_t  g_dropped_count = 0U;

// Extra stdio driver; its out_chars callback is set in wifi_log_init().
static stdio_driver_t     g_wifi_stdio_driver;

static void send_line (void);
static void write_chars (char const * p_buf, int len);

/*!
 * @brief Connects to Wi-Fi and starts mirroring stdio output over UDP.
 *
 * Call once, after stdio_init_all(). Retries the connection a few times (a
 * single failed attempt right after a reset is common), so this can block for
 * up to ~3x the per-attempt timeout. On failure, USB and other stdio output
 * still work as normal. Run wifi_log_listener.py on p_server_ip to see the
 * output without a USB cable.
 *
 * @param[in] p_ssid       Wi-Fi network name.
 * @param[in] p_password   Wi-Fi password.
 * @param[in] p_server_ip  IP address of the PC running the listener.
 * @param[in] server_port  UDP port the listener is on.
 * @return true if the Wi-Fi connection succeeded.
 */
bool
wifi_log_init (char const * p_ssid, char const * p_password,
               char const * p_server_ip, uint16_t server_port)
{
    bool b_ok = (0 == cyw43_arch_init());

    if (b_ok)
    {
        cyw43_arch_enable_sta_mode();

        // A single failed attempt is common right after a reset: a reset
        // doesn't send a clean 802.11 deauth, so some routers are slow to let
        // the same client reassociate until they time out the stale session.
        // Retrying absorbs that instead of needing a manual power cycle.
        int32_t result = -1;

        for (int32_t attempt = 0;
             (attempt < WIFI_LOG_CONNECT_RETRIES) && (0 != result);
             attempt++)
        {
            result = cyw43_arch_wifi_connect_timeout_ms(
                         p_ssid, p_password, CYW43_AUTH_WPA2_AES_PSK,
                         WIFI_LOG_CONNECT_TIMEOUT_MS);

            if (0 != result)
            {
                printf("Wi-Fi connect attempt %" PRId32 "/%d failed, "
                       "retrying...\n", attempt + 1, WIFI_LOG_CONNECT_RETRIES);
            }
        }

        b_ok = (0 == result);
    }

    if (b_ok)
    {
        b_ok = (0 != ip4addr_aton(p_server_ip, &g_server_addr));
    }

    if (b_ok)
    {
        g_server_port = server_port;

        cyw43_arch_lwip_begin();
        g_p_udp_pcb = udp_new();
        cyw43_arch_lwip_end();

        b_ok = (NULL != g_p_udp_pcb);
    }

    if (b_ok)
    {
        g_wifi_stdio_driver.out_chars = &write_chars;
        stdio_set_driver_enabled(&g_wifi_stdio_driver, true);
    }

    return b_ok;
}

/*!
 * @brief Gets how many lines couldn't be sent since boot.
 *
 * Lines are dropped when a packet buffer can't be allocated, usually under
 * bursty printing. If this climbs, raise MEM_SIZE in lwipopts.h.
 *
 * @return Number of dropped lines.
 */
uint32_t
wifi_log_get_dropped_count (void)
{
    return g_dropped_count;
}

/*!
 * @brief Sends the buffered line as one UDP packet and empties the buffer.
 */
static void
send_line (void)
{
    if ((0U != g_line_len) && (NULL != g_p_udp_pcb))
    {
        // Cast: g_line_len is below WIFI_LOG_LINE_BUF_SIZE, so it fits in
        // lwIP's 16-bit length type.
        struct pbuf * p_pbuf = pbuf_alloc(PBUF_TRANSPORT, (u16_t)g_line_len,
                                          PBUF_RAM);

        if (NULL != p_pbuf)
        {
            memcpy(p_pbuf->payload, g_line_buf, g_line_len);
            cyw43_arch_lwip_begin();
            udp_sendto(g_p_udp_pcb, p_pbuf, &g_server_addr, g_server_port);
            cyw43_arch_lwip_end();
            pbuf_free(p_pbuf);
        }
        else
        {
            g_dropped_count++;
        }
    }

    g_line_len = 0U;
}

/*!
 * @brief stdio out_chars callback: runs for every printf()/putchar().
 *
 * Buffers characters until a newline (or the buffer fills), then sends the
 * line as one UDP packet. The len parameter is an int because the SDK's
 * stdio_driver_t callback signature requires it.
 */
static void
write_chars (char const * p_buf, int len)
{
    for (int32_t idx = 0; idx < len; idx++)
    {
        char character = p_buf[idx];

        if ('\n' == character)
        {
            send_line();
        }
        else if ('\r' == character)
        {
            // Carriage returns are dropped.
        }
        else
        {
            if (g_line_len >= (sizeof(g_line_buf) - 1U))
            {
                send_line();
            }

            g_line_buf[g_line_len] = character;
            g_line_len++;
        }
    }
}

/*** end of file ***/

