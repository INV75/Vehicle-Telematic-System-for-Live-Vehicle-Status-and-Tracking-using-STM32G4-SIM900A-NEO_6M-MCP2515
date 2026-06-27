/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Real-time Vehicle Telemetry & Mapping Tracking System
  * @hardware       : STM32G491RE, SIM900A (UART1), NEO-6M (UART4), OLED (I2C1)
  * @platform       : Ubidots STEM Integration
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define RX_BUF_SIZE 512
#define SSD1306_I2C_ADDR 0x78

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart4;

volatile char sim_rx_buffer[RX_BUF_SIZE];
volatile uint16_t sim_rx_index = 0;
uint8_t sim_rx_byte;

/* 5x7 Font Matrix for OLED Display */
const uint8_t Font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5F, 0x00, 0x00}, {0x00, 0x07, 0x00, 0x07, 0x00}, {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62}, {0x36, 0x49, 0x55, 0x22, 0x50}, {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00}, {0x00, 0x41, 0x22, 0x1C, 0x00}, {0x14, 0x08, 0x3E, 0x08, 0x14}, {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08}, {0x00, 0x60, 0x60, 0x00, 0x00}, {0x20, 0x10, 0x08, 0x04, 0x02},
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00}, {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}, {0x00, 0x36, 0x36, 0x00, 0x00}, {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00}, {0x24, 0x24, 0x24, 0x24, 0x24}, {0x00, 0x41, 0x22, 0x14, 0x08}, {0x02, 0x01, 0x51, 0x09, 0x06},
    {0x32, 0x49, 0x79, 0x41, 0x3E}, {0x7C, 0x12, 0x11, 0x12, 0x7C}, {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01}, {0x3E, 0x41, 0x49, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01}, {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40}, {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06}, {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01}, {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63}, {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);

void OLED_WriteCmd(uint8_t cmd);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_PrintString(const char* str, uint8_t page, uint8_t col);

void LED_Pulse(uint8_t count, uint32_t delay_ms);
void Flush_SIM_Buffer(void);
uint8_t send_sim_cmd(char *cmd, char *expected_response, uint32_t timeout);
uint8_t Ensure_Network(void);
uint8_t send_http_post(char *url, char *body);
void fetch_sim_time(char *date_out, char *time_out);
uint8_t fetch_gps_decimal(double *lat_out, double *lon_out);
double convert_nmea_to_decimal(const char* nmea_str, char direction);

/* USER CODE BEGIN 0 */

/* ================= LED SIGNALS ================= */
void LED_Pulse(uint8_t count, uint32_t delay_ms) {
    for (int i = 0; i < count; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(delay_ms);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        if (i < count - 1) HAL_Delay(delay_ms);
    }
}

/* ================= OLED CORE DRIVERS ================= */
void OLED_WriteCmd(uint8_t cmd) {
    HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x00, 1, &cmd, 1, 20);
}

void OLED_Init(void) {
    HAL_Delay(100);
    OLED_WriteCmd(0xAE); OLED_WriteCmd(0x20); OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xB0); OLED_WriteCmd(0x00); OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xC8); OLED_WriteCmd(0xA1); OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14); OLED_WriteCmd(0xAF);
    HAL_Delay(100);
    OLED_Clear();
}

void OLED_Clear(void) {
    uint8_t blank_buffer[128] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        OLED_WriteCmd(0xB0 + i);
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);
        HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x40, 1, blank_buffer, 128, 50);
    }
}

void OLED_PrintString(const char* str, uint8_t page, uint8_t col) {
    while (*str) {
        char c = *str++;
        if (c < 32 || c > 90) {
            if (c >= 'a' && c <= 'z') c -= 32; else c = ' ';
        }
        uint8_t font_idx = c - 32;
        OLED_WriteCmd(0xB0 + page);
        OLED_WriteCmd(col & 0x0F);
        OLED_WriteCmd(0x10 | ((col >> 4) & 0x0F));
        HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x40, 1, (uint8_t*)Font5x7[font_idx], 5, 20);
        col += 6;
        if (col > 122) break;
    }
}

/* ================= CONVERT RAW NMEA TO MAP DEGREES ================= */
double convert_nmea_to_decimal(const char* nmea_str, char direction) {
    if (strlen(nmea_str) == 0) return 0.0;
    double raw_val = atof(nmea_str);
    int degrees = (int)(raw_val / 100);
    double minutes = raw_val - (degrees * 100);
    double decimal_val = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal_val = -decimal_val;
    }
    return decimal_val;
}

/* ================= MODEM COMMS & FAULT WATCHDOG ================= */
void Flush_SIM_Buffer(void) {
    __disable_irq();
    memset((void*)sim_rx_buffer, 0, RX_BUF_SIZE);
    sim_rx_index = 0;
    __enable_irq();
}

uint8_t send_sim_cmd(char *cmd, char *expected_response, uint32_t timeout) {
    Flush_SIM_Buffer();

    // Check and resolve hardware Overrun Errors before transmitting
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    __HAL_UART_FLUSH_DRREGISTER(&huart1);

    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout) {
        if (strstr((const char*)sim_rx_buffer, expected_response) != NULL) {
            HAL_Delay(100); // Allow hardware trailing lines to write out safely
            return 1;
        }

        // Auto-recover USART interrupt structure if broken by noise line spikes
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
            HAL_UART_Receive_IT(&huart1, &sim_rx_byte, 1);
        }

        if (strstr((const char*)sim_rx_buffer, "ERROR") != NULL && strstr(cmd, "HTTPACTION") == NULL) {
            return 0;
        }
    }
    return 0;
}

uint8_t Ensure_Network(void) {
    if (!send_sim_cmd("AT+SAPBR=2,1\r\n", "+SAPBR: 1,1", 2000)) {
        OLED_PrintString("NET DROP ALERT! ", 7, 0);
        send_sim_cmd("AT+SAPBR=0,1\r\n", "OK", 2000);
        send_sim_cmd("AT+CGATT=1\r\n", "OK", 4000);

        if(send_sim_cmd("AT+SAPBR=1,1\r\n", "OK", 5000)) {
            OLED_PrintString("NET RECONNECTED!", 7, 0);
            return 1;
        } else {
            OLED_PrintString("NET LINK FATAL  ", 7, 0);
            return 0;
        }
    }
    return 1;
}

/* ================= GPS TELEMETRY FETCHING ================= */
uint8_t fetch_gps_decimal(double *lat_out, double *lon_out) {
    char rx_buf[128] = {0};
    uint16_t rx_idx = 0;
    uint8_t rx_byte;
    uint32_t start_tick = HAL_GetTick();

    __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF);
    __HAL_UART_FLUSH_DRREGISTER(&huart4);

    OLED_PrintString("POLLING GPS FIX ", 0, 0);

    while ((HAL_GetTick() - start_tick) < 1500) {
        if (HAL_UART_Receive(&huart4, &rx_byte, 1, 5) == HAL_OK) {
            if (rx_idx < 127) {
                rx_buf[rx_idx++] = rx_byte;
                rx_buf[rx_idx] = '\0';
            }

            if (rx_byte == '\n') {
                char *gga = strstr(rx_buf, "$GPGGA,");
                if (gga != NULL) {
                    char *time_end = strchr(gga + 7, ',');
                    if (time_end) {
                        char *lat_start = time_end + 1;
                        char *lat_end = strchr(lat_start, ',');

                        if (lat_end && (lat_end - lat_start > 1)) {
                            char *ns_start = lat_end + 1;
                            char *ns_end = strchr(ns_start, ',');
                            char *lon_start = ns_end + 1;
                            char *lon_end = strchr(lon_start, ',');

                            if (lon_end && (lon_end - lon_start > 1)) {
                                char *ew_start = lon_end + 1;

                                char temp_lat[15] = {0};
                                char temp_lon[15] = {0};
                                strncpy(temp_lat, lat_start, lat_end - lat_start);
                                strncpy(temp_lon, lon_start, lon_end - lon_start);

                                *lat_out = convert_nmea_to_decimal(temp_lat, ns_start[0]);
                                *lon_out = convert_nmea_to_decimal(temp_lon, ew_start[0]);

                                OLED_PrintString("SATELLITE SYNCED", 0, 0);
                                return 1; // Valid coordinates achieved
                            }
                        }
                    }
                }
                rx_idx = 0;
                rx_buf[0] = '\0';
            }
        }
    }
    OLED_PrintString("SEARCHING SKY... ", 0, 0);
    return 0; // Fix not acquired yet
}

void fetch_sim_time(char *date_out, char *time_out) {
    if (send_sim_cmd("AT+CCLK?\r\n", "OK", 2000)) {
        char *p = strstr((const char*)sim_rx_buffer, "+CCLK: \"");
        if (p != NULL) {
            p += 8;
            if(strlen(p) >= 17) {
                sprintf(date_out, "%c%c-%c%c-20%c%c", p[6], p[7], p[3], p[4], p[0], p[1]);
                sprintf(time_out, "%c%c:%c%c:%c%c", p[9], p[10], p[12], p[13], p[15], p[16]);
                return;
            }
        }
    }
    strcpy(date_out, "00-00-2026");
    strcpy(time_out, "00:00:00");
}

/* ================= UBIDOTS TELEMETRY UPLOAD ================= */
uint8_t send_http_post(char *url, char *body) {
    char cmd[350];
    OLED_PrintString("STREAMING DATA..", 0, 0);

    send_sim_cmd("AT+HTTPTERM\r\n", "OK", 1000);
    HAL_Delay(100);

    if (!send_sim_cmd("AT+HTTPINIT\r\n", "OK", 2000)) {
        OLED_PrintString("STACK RST FAIL  ", 7, 0);
        return 0;
    }

    send_sim_cmd("AT+HTTPPARA=\"CID\",1\r\n", "OK", 1000);

    sprintf(cmd, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", url);
    if (!send_sim_cmd(cmd, "OK", 2000)) {
        OLED_PrintString("URL REJECTED    ", 7, 0);
        send_sim_cmd("AT+HTTPTERM\r\n", "OK", 1000);
        return 0;
    }

    send_sim_cmd("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", "OK", 1000);

    sprintf(cmd, "AT+HTTPDATA=%d,8000\r\n", (int)strlen(body));
    if (send_sim_cmd(cmd, "DOWNLOAD", 3000)) {
        if (send_sim_cmd(body, "OK", 4000)) {

            OLED_PrintString("WAITING TOWER RE", 7, 0);

            if(send_sim_cmd("AT+HTTPACTION=1\r\n", "HTTPACTION:", 45000)) {
                HAL_Delay(200);

                char *ptr = strstr((const char*)sim_rx_buffer, "HTTPACTION:");
                if (ptr) {
                    char *comma = strchr(ptr, ',');
                    if (comma) {
                        comma++;
                        while (*comma == ' ') comma++;

                        char code[4] = {0};
                        strncpy(code, comma, 3);

                        if (strncmp(code, "200", 3) == 0) {
                            OLED_PrintString("STREAM OK!      ", 0, 0);
                            OLED_PrintString("HTTP CODE: 200  ", 7, 0);
                            send_sim_cmd("AT+HTTPTERM\r\n", "OK", 1000);
                            return 1;
                        } else {
                            char msg[17];
                            sprintf(msg, "HTTP FAIL: %s ", code);
                            OLED_PrintString(msg, 7, 0);
                        }
                    }
                }
            } else {
                OLED_PrintString("POST TIMEOUT!   ", 0, 0);
                OLED_PrintString("NO TOWER ACK    ", 7, 0);
            }
        }
    }

    send_sim_cmd("AT+HTTPTERM\r\n", "OK", 1000);
    return 0;
}
/* USER CODE END 0 */

int main(void)
{
  uint32_t telemetry_counter = 0;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();

  OLED_Init();
  OLED_PrintString("[ SYSTEM START ]", 0, 0);
  LED_Pulse(3, 150);

  HAL_UART_Receive_IT(&huart1, &sim_rx_byte, 1);

  /* VERBOSE SYSTEM INITIALIZATION SEQUENCE ON OLED */
  int attempts = 0;
  OLED_PrintString("1. WAKING MODEM ", 2, 0);
  while (!send_sim_cmd("AT\r\n", "OK", 1000) && attempts < 10) {
      attempts++;
      HAL_Delay(100);
  }

  OLED_PrintString("2. ECHO OFF     ", 3, 0);
  send_sim_cmd("ATE0\r\n", "OK", 1000);
  send_sim_cmd("AT+CLTS=1\r\n", "OK", 1000);

  OLED_PrintString("3. CELL NETWORK ", 4, 0);
  while (!send_sim_cmd("AT+CREG?\r\n", "+CREG: 0,1", 2000) &&
         !send_sim_cmd("AT+CREG?\r\n", "+CREG: 0,5", 2000)) {
      LED_Pulse(1, 50);
      HAL_Delay(300);
  }

  OLED_PrintString("4. GPRS CONTEXT ", 5, 0);
  send_sim_cmd("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", "OK", 1000);
  send_sim_cmd("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"\r\n", "OK", 1000);

  OLED_PrintString("5. ATTACH BEARER", 6, 0);
  send_sim_cmd("AT+CGATT=1\r\n", "OK", 4000);

  if (!send_sim_cmd("AT+SAPBR=1,1\r\n", "OK", 5000)) {
      OLED_PrintString("GPRS INITIAL ERR", 7, 0);
      HAL_Delay(2000);
  } else {
      OLED_PrintString("HARDWARE READY! ", 7, 0);
      HAL_Delay(1200);
  }

  /* CORRECTED UBIDOTS ENDPOINT DEFINITION */
  char webhook_url[] = "http://industrial.api.ubidots.com/api/v1.6/devices/iot-vehicle-telemetry-bem/?token=BBUS-XZYmAoG2undEtAWIteIg7cDWYWOmcp";

  char dynamic_date[15] = "";
  char dynamic_time[15] = "";
  double current_lat = 0.0;
  double current_lon = 0.0;
  char json_payload[200];
  char screen_buf[24];

  OLED_Clear();

  while (1)
  {
      fetch_sim_time(dynamic_date, dynamic_time);
      uint8_t has_fix = fetch_gps_decimal(&current_lat, &current_lon);

      /* CONTINUOUS UI LAYOUT REFRESH */
      sprintf(screen_buf, "DATE: %s", dynamic_date);    OLED_PrintString(screen_buf, 1, 0);
      sprintf(screen_buf, "TIME: %s", dynamic_time);    OLED_PrintString(screen_buf, 2, 0);

      if (has_fix) {
          sprintf(screen_buf, "LAT: %.6f  ", current_lat); OLED_PrintString(screen_buf, 4, 0);
          sprintf(screen_buf, "LON: %.6f  ", current_lon); OLED_PrintString(screen_buf, 5, 0);
      } else {
          OLED_PrintString("LAT: WAITING FIX", 4, 0);
          OLED_PrintString("LON: WAITING FIX", 5, 0);
      }

      sprintf(screen_buf, "STREAM TX: #%-5lu", telemetry_counter);
      OLED_PrintString(screen_buf, 6, 0);

      /* REAL-TIME TELEMETRY STREAMING ACTION BLOCK (UBIDOTS GEOLOCATION FORMAT) */
      if (has_fix) {
          // Explicit nested syntax structure matching standard Ubidots tracking expectations
          snprintf(json_payload, sizeof(json_payload),
                   "{\"location\":{\"value\":1,\"context\":{\"lat\":%.6f,\"lng\":%.6f}}}",
                   current_lat, current_lon);

          Ensure_Network();

          if (send_http_post(webhook_url, json_payload)) {
              telemetry_counter++;
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
              HAL_Delay(1000); // Visual signal confirming dashboard acceptance
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
          } else {
              LED_Pulse(5, 70); // Fault indicator flash patterns
          }
      }

      HAL_Delay(1000); // System cycle latency control delay
  }
  return 0;
}

/* ================= HARDWARE INTERRUPT CALLBACK RESCUE ================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (sim_rx_index >= RX_BUF_SIZE - 2) {
            sim_rx_index = 0;
        }
        sim_rx_buffer[sim_rx_index++] = sim_rx_byte;
        ((char*)sim_rx_buffer)[sim_rx_index] = '\0';

        HAL_UART_Receive_IT(&huart1, &sim_rx_byte, 1);
    }
}

/* ================= SYSTEM RCC SYSTEM CLOCK CONFIGURATION ================= */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart1);
}

static void MX_UART4_Init(void) {
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart4);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
