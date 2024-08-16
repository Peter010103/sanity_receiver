#define UART_NUM UART_NUM_2
#define UART_TX_PIN (43)
#define UART_RX_PIN (44)
#define UART_BUF_SIZE 1024

#define SBUS_PACKET_SIZE 25
#define SBUS_RATE 100
#define SBUS_DELAY_MS (1000 / SBUS_RATE)
#define SBUS_INVERT (0)

#define CHANNEL_MIN 1000
#define CHANNEL_MAX 2000
#define UART_MIN 192
#define UART_MAX 1793

#define WIFI_SSID "robotlab"
#define WIFI_PASS "**r0b0t**"

#define UDP_PORT 10240
#define UDP_BUF_SIZE 2048

#define SWMC_STREAM_ID 777
#define SWMC_FAST_DECODE (0)

#define DEBUG_PRINT_ENABLED (0)
#define DEBUG_UDP_ENABLED (0)

#if DEBUG_UDP_ENABLED
#define NUM_STAT_PACKETS 100
uint16_t *time_intervals = NULL; 
uint8_t packet_counter = 0;

#define UDP_CLIENT_TARGET_IP "255.255.255.255" // Target IP address of listener
#define UDP_CLIENT_TARGET_PORT 10250
#endif

uint8_t MY_ID = 1;

#define STATIC_IP_ADDR "10.0.0.101"
#define STATIC_GATEWAY "10.0.0.1"
#define STATIC_NETMASK "255.255.255.0"
