/** @file wifi_log.h
 *
 * @brief Mirrors printf() output over Wi-Fi as UDP packets.
 *
 * FOR TESTING PURPOSES ONLY, TO REPLACE LATER.
 */

#ifndef WIFI_LOG_H
#define WIFI_LOG_H

#include <stdbool.h>
#include <stdint.h>

bool     wifi_log_init (char const * p_ssid, char const * p_password,
                        char const * p_server_ip, uint16_t server_port);
uint32_t wifi_log_get_dropped_count (void);

#endif /* WIFI_LOG_H */

/*** end of file ***/

