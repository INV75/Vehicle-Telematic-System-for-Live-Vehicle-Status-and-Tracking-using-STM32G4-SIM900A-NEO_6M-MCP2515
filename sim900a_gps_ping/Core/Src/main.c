/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32G491RE + SIM900A Fast Telemetry System
  * @note           : Optimized for fast boot and stable webhook POSTing
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define RX_BUF_SIZE 512
#define SSD1306_I2C_ADDR 0x78

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

uint8_t rx_buffer[RX_BUF_SIZE];
volatile uint16_t rx_index = 0;
uint8_t rx_byte;

/* Complete 5x7 Font Matrix */
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

void OLED_WriteCmd(uint8_t cmd);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_PrintString(const char* str, uint8_t page, uint8_t col);
void Flush_Buffer(void);
HAL_StatusTypeDef Send_AT_Command(const char* cmd, const char* expected_resp, uint32_t timeout);
uint8_t Wait_For_Network(void);
void Init_GPRS(void);

/* USER CODE BEGIN 0 */
void OLED_WriteCmd(uint8_t cmd) {
    HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x00, 1, &cmd, 1, 20);
}

void OLED_Init(void) {
    HAL_Delay(150);
    OLED_WriteCmd(0xAE); OLED_WriteCmd(0x20); OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xB0); OLED_WriteCmd(0x00); OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xC8); OLED_WriteCmd(0xA1); OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14); OLED_WriteCmd(0xAF);
    HAL_Delay(100);
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

void Flush_Buffer(void) {
    __disable_irq(); // Prevent race conditions
    memset((void*)rx_buffer, 0, RX_BUF_SIZE);
    rx_index = 0;
    __enable_irq();
}

HAL_StatusTypeDef Send_AT_Command(const char* cmd, const char* expected_resp, uint32_t timeout) {
    Flush_Buffer();
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 500);

    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < timeout) {
        if (strstr((char*)rx_buffer, expected_resp) != NULL) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(30);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

uint8_t Wait_For_Network(void) {
    OLED_Clear();
    OLED_PrintString("SEARCHING NET...", 0, 0);

    // Poll CREG every 1.5 seconds until registered (Home network = 1, Roaming = 5)
    uint32_t start_time = HAL_GetTick();
    while((HAL_GetTick() - start_time) < 45000) {
        if (Send_AT_Command("AT+CREG?\r\n", "+CREG: 0,1", 1000) == HAL_OK ||
            Send_AT_Command("AT+CREG?\r\n", "+CREG: 0,5", 1000) == HAL_OK) {
            OLED_PrintString("NETWORK OK!     ", 2, 0);
            return 1;
        }
        HAL_Delay(1000);
    }
    OLED_PrintString("NET TIMEOUT!    ", 2, 0);
    return 0;
}

void Init_GPRS(void) {
    OLED_PrintString("CONFIG GPRS...  ", 4, 0);
    Send_AT_Command("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n", "OK", 1000);
    Send_AT_Command("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"\r\n", "OK", 1000);

    // Only open the bearer if it isn't already open
    if (Send_AT_Command("AT+SAPBR=2,1\r\n", "+SAPBR: 1,1", 1000) != HAL_OK) {
        OLED_PrintString("OPENING BEARER..", 6, 0);
        Send_AT_Command("AT+SAPBR=1,1\r\n", "OK", 5000);
    }
    HAL_Delay(1000);
}
/* USER CODE END 0 */

int main(void)
{
  char dev_status[16] = "IDLE";
  char signal_strength[6] = "0";
  char lat_str[16] = "0.0000";
  char lon_str[16] = "0.0000";
  char current_time[12] = "00:00:00";
  char current_date[16] = "01-01-2026";
  char screen_buf[24];
  char http_payload[350];
  char cmd_buffer[128];

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  OLED_Init();

  // Give the module hardware time to stabilize
  HAL_Delay(1000);

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  // Sync baud rate and disable echo
  Send_AT_Command("AT\r\n", "OK", 500);
  Send_AT_Command("ATE0\r\n", "OK", 500);

  /* CRITICAL STEP: Wait for network BEFORE doing anything else */
  if (Wait_For_Network()) {
      Init_GPRS();
  }

  OLED_Clear();

    while (1)
    {
        /* 1. Network & Signal Check */
        if (Send_AT_Command("AT+CREG?\r\n", "OK", 1000) == HAL_OK) {
            if (strstr((char*)rx_buffer, ",1") || strstr((char*)rx_buffer, ",5")) strcpy(dev_status, "CONNECTED");
            else strcpy(dev_status, "SEARCHING");
        }

        if (Send_AT_Command("AT+CSQ\r\n", "OK", 1000) == HAL_OK) {
            char *csq_ptr = strstr((char*)rx_buffer, "+CSQ:");
            if (csq_ptr != NULL) {
                int rssi = 0;
                sscanf(csq_ptr, "+CSQ: %d", &rssi);
                sprintf(signal_strength, "%d", rssi);
            }
        }

        /* 2. Check IP Address status & Recover if needed */
        if (Send_AT_Command("AT+SAPBR=2,1\r\n", "+SAPBR: 1,1", 1000) != HAL_OK) {
             // Lightweight recovery. Don't tear down CGATT.
             Send_AT_Command("AT+SAPBR=0,1\r\n", "OK", 1000);
             Send_AT_Command("AT+SAPBR=1,1\r\n", "OK", 4000);
        }

        /* 3. Fetch LBS Location */
        Flush_Buffer();
        HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CIPGSMLOC=1,1\r\n", 18, 100);
        uint32_t tick = HAL_GetTick();
        uint8_t loc_success = 0;

        while ((HAL_GetTick() - tick) < 5000) {
            if (strstr((char*)rx_buffer, "+CIPGSMLOC:") != NULL && strstr((char*)rx_buffer, "OK") != NULL) {
                loc_success = 1;
                break;
            }
        }

        if (loc_success) {
            char *loc_ptr = strstr((char*)rx_buffer, "+CIPGSMLOC:");
            int res_code = -1;
            if (loc_ptr != NULL) {
                char temp_date[12] = {0};
                if (sscanf(loc_ptr, "+CIPGSMLOC: %d,%15[^,],%15[^,],%11[^,],%11[^\r\n]",
                           &res_code, lon_str, lat_str, temp_date, current_time) >= 5) {
                    if (res_code == 0) {
                        int yyyy, mm, dd;
                        if (sscanf(temp_date, "%d/%d/%d", &yyyy, &mm, &dd) == 3) {
                            sprintf(current_date, "%02d-%02d-%04d", dd, mm, yyyy);
                        }
                    }
                }
            }
        }

        /* 4. Update Time from network */
        if (Send_AT_Command("AT+CCLK?\r\n", "OK", 1000) == HAL_OK) {
            char *cclk_ptr = strstr((char*)rx_buffer, "+CCLK:");
            if (cclk_ptr != NULL) {
                int yy, mm, dd;
                char temp_time[12] = {0};
                if (sscanf(cclk_ptr, "+CCLK: \"%d/%d/%d,%8[^+]\"", &yy, &mm, &dd, temp_time) == 4) {
                    sprintf(current_date, "%02d-%02d-20%02d", dd, mm, yy);
                    strcpy(current_time, temp_time);
                }
            }
        }

        /* 5. UI Update */
        OLED_Clear();
        sprintf(screen_buf, "STATUS: %s", dev_status);
        OLED_PrintString(screen_buf, 0, 0);
        sprintf(screen_buf, "RSSI:   %s", signal_strength);
        OLED_PrintString(screen_buf, 2, 0);
        sprintf(screen_buf, "LAT:    %s", lat_str);
        OLED_PrintString(screen_buf, 4, 0);
        sprintf(screen_buf, "LON:    %s", lon_str);
        OLED_PrintString(screen_buf, 5, 0);
        sprintf(screen_buf, "DATE:   %s", current_date);
        OLED_PrintString(screen_buf, 6, 0);
        sprintf(screen_buf, "TIME:   %s", current_time);
        OLED_PrintString(screen_buf, 7, 0);

        /* 6. Robust HTTP POST */
        Send_AT_Command("AT+HTTPTERM\r\n", "OK", 500); // Always terminate previous hanging sessions

        if (Send_AT_Command("AT+HTTPINIT\r\n", "OK", 2000) == HAL_OK) {

            Send_AT_Command("AT+HTTPPARA=\"CID\",1\r\n", "OK", 1000);
            Send_AT_Command("AT+HTTPPARA=\"URL\",\"http://webhook.site/53020a65-7237-442e-9d17-a387ab8e596e\"\r\n", "OK", 1000);
            Send_AT_Command("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", "OK", 1000);

            sprintf(http_payload,
                    "{\"device_status\":\"%s\",\"signal_strength\":%s,\"gps_position_coordinates\":{\"latitude\":\"%s\",\"longitude\":\"%s\"},\"date\":\"%s\",\"time\":\"%s\"}",
                    dev_status, signal_strength, lat_str, lon_str, current_date, current_time);

            sprintf(cmd_buffer, "AT+HTTPDATA=%d,5000\r\n", (int)strlen(http_payload));

            if (Send_AT_Command(cmd_buffer, "DOWNLOAD", 2000) == HAL_OK) {

                // Write payload
                if (Send_AT_Command(http_payload, "OK", 5000) == HAL_OK) {

                    // Execute POST (1 = POST)
                    if(Send_AT_Command("AT+HTTPACTION=1\r\n", "+HTTPACTION: 1", 10000) == HAL_OK) {
                        char *action_ptr = strstr((char*)rx_buffer, "+HTTPACTION: 1,");
                        if (action_ptr != NULL) {
                            int http_code = 0;
                            sscanf(action_ptr, "+HTTPACTION: 1,%d", &http_code);

                            // Visual feedback if 200 OK was received
                            if (http_code >= 200 && http_code < 300) {
                                OLED_PrintString(" * ", 0, 110);
                            }
                        }
                    }
                }
            }
            Send_AT_Command("AT+HTTPTERM\r\n", "OK", 1000);
        }

        /* 7. Interval Rest */
        HAL_Delay(12000);
    }
}

/** HAL Clock Configurations remain completely untouched below **/

void SystemClock_Config(void)
{
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_index < RX_BUF_SIZE - 1) {
            rx_buffer[rx_index++] = rx_byte;
            rx_buffer[rx_index] = '\0';
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
