// FOR TESTING PURPOSES ONLY, TO REPLACE LATER

#pragma once

// Minimal lwIP config for wireless debug logging over UDP + DHCP.
// TCP/sockets are switched off since this project doesn't need them --
// keeping lwIP's footprint and config surface small.

#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
// wifi_log.c allocates a PBUF_RAM per printed line; under bursty printing
// several can be in flight (queued for transmit) at once before the driver
// frees them, so this needs real headroom -- too small and pbuf_alloc()
// silently fails and that line is just dropped. RP2040 has 264KB SRAM, so
// this is cheap.
#define MEM_SIZE                    16000

#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1

#define LWIP_IPV4                   1
#define LWIP_UDP                    1
#define LWIP_TCP                    0
#define LWIP_DHCP                   1
#define LWIP_DNS                    0

#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1

#define MEMP_NUM_UDP_PCB            4
#define PBUF_POOL_SIZE              16

#define LWIP_STATS                  0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0

// Needed by cyw43_arch when lwIP is built without an RTOS.
#define LWIP_PROVIDE_ERRNO          1
