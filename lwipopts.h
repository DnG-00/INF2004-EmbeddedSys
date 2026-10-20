#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4

// wifi_log.c allocates a PBUF_RAM per printed line; under bursty printing
// several can be in flight (queued for transmit) at once before the driver
// frees them, so this needs real headroom -- too small and pbuf_alloc()
// silently fails and that line is just dropped. The RP2040 has 264 KB SRAM,
// so this is cheap.
#define MEM_SIZE                    16000   

#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DHCP                   1
#define LWIP_DNS                    1
#define LWIP_TCP_KEEPALIVE          1
#define LWIP_NETCONN                0
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define MEMP_NUM_UDP_PCB            4
#define PBUF_POOL_SIZE              16
#define LWIP_STATS                  0
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0
#define LWIP_CHKSUM_ALGORITHM       3
#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define MEM_STATS                   0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0
#define MEMP_NUM_SYS_TIMEOUT   (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 3)

// Needed by cyw43_arch when lwIP is built without an RTOS.
#define LWIP_PROVIDE_ERRNO          1

#endif

/*** end of file ***/