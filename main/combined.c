#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "sys/time.h"

#include "config.h"

// Initialize channel values
uint16_t channels[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500,
                         1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Logging macro
#if DEBUG_PRINT_ENABLED
#define DEBUG_LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define DEBUG_LOGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#define DEBUG_LOGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#else
#define DEBUG_LOGI(tag, format, ...)
#define DEBUG_LOGE(tag, format, ...)
#define DEBUG_LOGW(tag, format, ...)
#endif

struct routing_header {
  uint8_t destination_host;
  uint8_t destination_process;
  uint16_t source_host;
  uint16_t source_process;
  uint64_t timestamp;
  uint8_t meta;
  uint8_t next_header;
}; // 16 bytes

struct stream_header {
  uint8_t source_host;
  uint16_t source_process;
  uint32_t encoded_stream_sequence;
  uint8_t next_header;
}; // 8 bytes

struct payload {
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;
  uint16_t thrust;
  uint8_t flags;
}; // 9 bytes

struct swmc_packet {
  struct routing_header routing_header;
  struct stream_header stream_header;
  struct payload *payload_array; // Array of payloads per agent
  uint16_t payload_count;        // Number of agents
};

static char rx_buffer[UDP_BUF_SIZE];

esp_err_t setup_uart(void) {
  const uart_config_t uart_config = {.baud_rate = 100000,
                                     .data_bits = UART_DATA_8_BITS,
                                     .parity = UART_PARITY_EVEN,
                                     .stop_bits = UART_STOP_BITS_2,
                                     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

  esp_err_t err;

  err = uart_param_config(UART_NUM, &uart_config);
  if (err != ESP_OK) {
    DEBUG_LOGE("UART", "uart_param_config failed");
    return err;
  }

  err = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    DEBUG_LOGE("UART", "uart_set_pin failed");
    return err;
  }

  err = uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
  if (err != ESP_OK) {
    DEBUG_LOGE("UART", "uart_driver_install failed");
    return err;
  }

#if SBUS_INVERT
  err = uart_set_line_inverse(UART_NUM, UART_INVERSE_TXD | UART_INVERSE_RXD);
  if (err != ESP_OK) {
    DEBUG_LOGE("UART", "uart_set_line_inverse failed");
    return err;
  }
#endif

  DEBUG_LOGI("UART", "uart setup successful");

  return ESP_OK;
}

uint16_t map_to_uart(uint16_t channel_value) {
  return UART_MIN + ((channel_value - CHANNEL_MIN) * (UART_MAX - UART_MIN)) /
                        (CHANNEL_MAX - CHANNEL_MIN);
}

void construct_sbus(uint16_t *channels, uint8_t *sbus_packet) {
  /*
   * SBUS Packet Format:
   * Byte[0]:  SBUS header, 0x0F
   * Bytes[1-22]:  16 servo channels, 11 bits each
   * Byte[23]:
   *   Bit 0:  channel 17 (0x01)
   *   Bit 1:  channel 18 (0x02)
   *   Bit 2:  frame lost (0x04)
   *   Bit 3:  failsafe activated (0x08)
   * Byte[24]:  SBUS footer
   */

  // Header
  sbus_packet[0] = 0x0F;
  memset(sbus_packet + 1, 0, 22); // Clear data to ensure no leftover bits

  // 16 channels, 11 bits each
  for (int i = 0; i < 16; i++) {
    int byte_index = 1 + i * 11 / 8;
    int bit_index = (i * 11) % 8;

    uint16_t value = map_to_uart(channels[i]);

    sbus_packet[byte_index] |= (value << bit_index) & 0xFF;
    sbus_packet[byte_index + 1] |= (value >> (8 - bit_index)) & 0xFF;

    if (bit_index > 5) {
      sbus_packet[byte_index + 2] |= (value >> (16 - bit_index)) & 0xFF;
    }
  }

  // Flags: channels 17, 18, frame lost, failsafe activated
  // Set as needed, e.g., packet[23] |= 0x01 for channel 17
  sbus_packet[23] = 0x00;

  // Footer
  sbus_packet[24] = 0x00;
}

void send_sbus(void *pvParameters) {
  uint8_t sbus_packet[SBUS_PACKET_SIZE] = {0};

  while (1) {
    construct_sbus(channels, sbus_packet);
    uart_write_bytes(UART_NUM, (const char *)sbus_packet, SBUS_PACKET_SIZE);
    vTaskDelay(pdMS_TO_TICKS(SBUS_DELAY_MS));
  }
}

uint64_t ntohll(uint64_t val) {
  return ((val & 0xFF00000000000000ULL) >> 56) |
         ((val & 0x00FF000000000000ULL) >> 40) |
         ((val & 0x0000FF0000000000ULL) >> 24) |
         ((val & 0x000000FF00000000ULL) >> 8)  |
         ((val & 0x00000000FF000000ULL) << 8)  |
         ((val & 0x0000000000FF0000ULL) << 24) |
         ((val & 0x000000000000FF00ULL) << 40) |
         ((val & 0x00000000000000FFULL) << 56);
}

struct swmc_packet *decode_packet(const uint32_t stream_id, const char *buffer,
                                  int buffer_len) {
  if (buffer_len < 24) {
    DEBUG_LOGE("SWMC", "Obtained buffer must be at least long to hold a "
                       "routing and stream header");
    return NULL;
  }

  if (*((unsigned char *)&buffer[15]) != 0x05) {
    DEBUG_LOGE("SWMC", "This frame does not have a stream header immediately "
                       "after the routing header");
    return NULL;
  }

  if (*((unsigned char *)&buffer[23]) != 0xFF) {
    DEBUG_LOGE("SWMC", "This frame does not have the payload immediately after "
                       "the stream header");
    return NULL;
  }

  struct swmc_packet *swmc_packet = calloc(1, sizeof(struct swmc_packet));

  // Decode routing header, including endianess correction
  swmc_packet->routing_header.destination_host = buffer[0];
  swmc_packet->routing_header.destination_process =
      ntohs(*(uint16_t *)&buffer[1]);
  swmc_packet->routing_header.source_host = buffer[3];
  swmc_packet->routing_header.source_process = ntohs(*(uint16_t *)&buffer[4]);
  memcpy(&swmc_packet->routing_header.timestamp, &buffer[6], sizeof(uint64_t));
  swmc_packet->routing_header.timestamp =
      ntohll(swmc_packet->routing_header.timestamp);
  swmc_packet->routing_header.meta = buffer[14];
  swmc_packet->routing_header.next_header = buffer[15];

  // Decode stream header, including endianess correction
  swmc_packet->stream_header.source_host = buffer[16];
  swmc_packet->stream_header.source_process = ntohs(*(uint16_t *)&buffer[17]);
  swmc_packet->stream_header.encoded_stream_sequence =
      ntohl(*(uint32_t *)&buffer[19]);
  swmc_packet->stream_header.next_header = buffer[23];

  // Only attempt to decode the payload if the stream id matches
  if (stream_id != swmc_packet->stream_header.encoded_stream_sequence) {
    DEBUG_LOGI("SWMC", "Stream ID does not match %lu",
               swmc_packet->stream_header.encoded_stream_sequence);
    free(swmc_packet);
    return NULL;
  }

  // TODO: Fast decode
  uint16_t payload_count = (buffer_len - 24) / 9;
  DEBUG_LOGI("SWMC", "Obtained %d payload packets", payload_count);
  swmc_packet->payload_count = payload_count;

  swmc_packet->payload_array = calloc(payload_count, sizeof(struct payload));
  if (!swmc_packet->payload_array) {
    free(swmc_packet);
    return NULL;
  }

  // Decode all payload, including endianess correction
  uint16_t buf_pos = 24;
  for (uint16_t i = 0; i < payload_count; i++) {
    swmc_packet->payload_array[i].roll =
        ntohs(*(uint16_t *)&buffer[buf_pos + 0]);
    swmc_packet->payload_array[i].pitch =
        ntohs(*(uint16_t *)&buffer[buf_pos + 2]);
    swmc_packet->payload_array[i].yaw =
        ntohs(*(uint16_t *)&buffer[buf_pos + 4]);
    swmc_packet->payload_array[i].thrust =
        ntohs(*(uint16_t *)&buffer[buf_pos + 6]);
    swmc_packet->payload_array[i].flags = buffer[buf_pos + 8];

    // Move the buffer position by the size of the payload struct (9 bytes)
    buf_pos += 9;
  }

  return swmc_packet;
};

void decode_flags(const uint8_t aux_flags) {
  /*
   * Flag Format:
   * Byte[0, 1]: Aux 1, Aux 2
   * Byte[2-3, 4-5, 6-7]: Aux 3, Aux 4, Aux 5
   */

  static const uint16_t lookup_1bit[2] = {1000, 2000};
  static const uint16_t lookup_2bit[4] = {1000, 1350, 1650, 2000};

  for (size_t i = 0; i < 5; i++) {
    uint8_t flag_value;
    if (i < 2) {
      // Extract 1-bit flag value for aux 1 and 2
      flag_value = (aux_flags >> i) & 0x01;
      channels[i + 4] = lookup_1bit[flag_value];
    } else {
      // Extract 2-bit flag value for aux 3, 4, and 5
      flag_value = (aux_flags >> (2 * (i - 1))) & 0x03;
      channels[i + 4] = lookup_2bit[flag_value];
    }
  }
}

#if DEBUG_UDP_ENABLED
void udp_sender_task(void *pvParameters) {
  struct sockaddr_in client_addr;
  client_addr.sin_addr.s_addr = inet_addr(UDP_CLIENT_TARGET_IP);
  client_addr.sin_family = AF_INET;
  client_addr.sin_port = htons(UDP_CLIENT_TARGET_PORT);

  size_t payload_size = sizeof(MY_ID) + (NUM_STAT_PACKETS * sizeof(uint16_t));
  uint8_t *payload = (uint8_t *)malloc(payload_size);

  memcpy(payload, &MY_ID, sizeof(MY_ID));
  memcpy(payload + sizeof(MY_ID), time_intervals,
         NUM_STAT_PACKETS * sizeof(uint16_t));

  DEBUG_LOGI("TELEM", "Size of time_intervals: %d",
             sizeof(time_intervals) / sizeof(uint16_t));

  for (int i = 0; i < 5; i++) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
      DEBUG_LOGE(TAG, "Unable to create socket: errno %d", errno);
      vTaskDelete(NULL);
      return;
    }
    DEBUG_LOGI(TAG, "Socket created, sending to %s:%d", UDP_CLIENT_TARGET_IP,
               UDP_CLIENT_TARGET_PORT);

    int err = sendto(sock, payload, payload_size, 0,
                     (struct sockaddr *)&client_addr, sizeof(client_addr));
    if (err < 0) {
      DEBUG_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    } else {
      DEBUG_LOGI(TAG, "Message sent");
    }

    shutdown(sock, 0);
    close(sock);
  }

  vTaskDelete(NULL);
}
#endif

void udp_listener_task(void *pvParameters) {
  struct sockaddr_in server_addr;

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) {
    DEBUG_LOGE("UDP", "Unable to create socket: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }
  DEBUG_LOGI("UDP", "Socket created");

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(UDP_PORT);

  // TODO: Fix behaviour when packet given on wrong broadcast address
  if (inet_pton(AF_INET, "10.0.0.255", &server_addr.sin_addr) <= 0) {
    DEBUG_LOGE("UDP", "Invalid address: errno %d", errno);
    close(sock);
    vTaskDelete(NULL);
    return;
  }

  int err = bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (err < 0) {
    DEBUG_LOGE("UDP", "Socket unable to bind: errno %d", errno);
    close(sock);
    vTaskDelete(NULL);
    return;
  }
  DEBUG_LOGI("UDP", "UDP server task started, listening for packets...");

#if DEBUG_UDP_ENABLED
  uint16_t packet_counter = 0;
  TickType_t current_ticks;
  TickType_t last_ticks = xTaskGetTickCount();

  DEBUG_LOGI("UDP Testing", "%lu", portTICK_PERIOD_MS);
#endif

  while (1) {
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    memset(rx_buffer, 0, sizeof(rx_buffer));

    int rx_buffer_len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                                 (struct sockaddr *)&source_addr, &socklen);

#if DEBUG_UDP_ENABLED
    current_ticks = xTaskGetTickCount();
    uint16_t dt = (current_ticks - last_ticks) * portTICK_PERIOD_MS;
    DEBUG_LOGI("UDP Testing", "%hu", dt);

    time_intervals[packet_counter] = dt;
    last_ticks = current_ticks;

    if (packet_counter < NUM_STAT_PACKETS) {
      packet_counter++;

      if (packet_counter % 16 == 0) {
        ESP_LOGI("UDP Testing", "Received %d packets", packet_counter);
      }

    } else {
      break;
    }
#endif

    if (rx_buffer_len < 0) {
      DEBUG_LOGE("UDP", "recvfrom failed: errno %d", errno);
      break;
    }

    struct swmc_packet *swmc_packet =
        decode_packet(SWMC_STREAM_ID, rx_buffer, rx_buffer_len);

    if (!swmc_packet) {
      DEBUG_LOGE("UDP", "Failed to decode packet");
      return;
    }

    if (MY_ID > swmc_packet->payload_count - 1) {
      DEBUG_LOGE("UDP", "MY_ID (%d) is out of bounds. Payload count: %d", MY_ID,
                 swmc_packet->payload_count);
      return;
    }

    struct payload *my_payload = &((swmc_packet->payload_array)[MY_ID]);
    channels[0] = my_payload->roll;
    channels[1] = my_payload->pitch;
    channels[2] = my_payload->thrust;
    channels[3] = my_payload->yaw;
    decode_flags(my_payload->flags);
  }

  if (sock != -1) {
    DEBUG_LOGE("UDP", "Shutting down socket and restarting...");
    close(sock);
  }

#if DEBUG_UDP_ENABLED
  vTaskDelay(pdMS_TO_TICKS(400 * MY_ID));
  DEBUG_LOGI("TELEM", "Creating UDP sender");
  xTaskCreate(udp_sender_task, "udp_sender", 4096, NULL, 1, NULL);
#endif

  vTaskDelete(NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    DEBUG_LOGI("WIFI", "Wi-Fi STA started, connecting to AP...");
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    DEBUG_LOGI("WIFI", "Wi-Fi STA disconnected, retrying connection...");
    esp_wifi_connect();
  }
}

void wifi_init_sta(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  DEBUG_LOGI("WIFI", "Wi-Fi initialization complete");

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
      &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASS,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  DEBUG_LOGI("WIFI", "Wi-Fi configuration set to STA mode with SSID: %s",
             WIFI_SSID);

  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  ESP_ERROR_CHECK(esp_wifi_start());
  DEBUG_LOGI("WIFI", "Wi-Fi STA started");
}

void app_main(void) {
  ESP_ERROR_CHECK(nvs_flash_init());
  wifi_init_sta();

  ESP_ERROR_CHECK(setup_uart());
  xTaskCreate(udp_listener_task, "udp_listener", 8192, NULL, 2, NULL);
  xTaskCreate(send_sbus, "sbus_sender", 2048, NULL, 2, NULL);

}
