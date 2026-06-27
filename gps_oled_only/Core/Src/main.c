/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32G491RE + SIM900A LBS Coordinate Tracker (Debug Timing)
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
void Init_GPRS_Stack(void);

/* USER CODE BEGIN 0 */
void OLED_WriteCmd(uint8_t cmd) {
    HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x00, 1, &cmd, 1, 20);
}

void OLED_Init(void) {
    HAL_Delay(150);
    OLED_WriteCmd(0xAE);
    OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xB0);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0xA1);
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xAF);
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
            if (c >= 'a' && c <= 'z') c -= 32;
            else c = ' ';
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
    memset(rx_buffer, 0, RX_BUF_SIZE);
    rx_index = 0;
}

HAL_StatusTypeDef Send_AT_Command(const char* cmd, const char* expected_resp, uint32_t timeout) {
    Flush_Buffer();
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);

    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < timeout) {
        if (strstr((char*)rx_buffer, expected_resp) != NULL) {
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

void Init_GPRS_Stack(void) {
    OLED_PrintString("INIT GPRS...      ", 0, 0);
    Send_AT_Command("AT+SAPBR=0,1\r\n", "OK", 2000);
    HAL_Delay(500);

    Send_AT_Command("AT+CGATT=0\r\n", "OK", 3000);
    HAL_Delay(1000);

    Send_AT_Command("AT+CGATT=1\r\n", "OK", 4000);
    Send_AT_Command("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n", "OK", 1000);

    // --> CHANGE APN HERE IF NOT AIRTEL ("bsnlnet" or "www") <--
    Send_AT_Command("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"\r\n", "OK", 1000);
    Send_AT_Command("AT+SAPBR=1,1\r\n", "OK", 5000);
}
/* USER CODE END 0 */

int main(void)
{
  char dev_status[16] = "IDLE";
  char lat_str[16] = "0.0000";
  char lon_str[16] = "0.0000";
  char screen_buf[24];

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  OLED_Init();
  OLED_Clear();
  OLED_PrintString("SYSTEM BOOTING", 0, 15);
  OLED_PrintString("CONNECTING MOD...", 2, 5);

  /* Start UART Interrupt */
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  /* Wait for SIM900A to respond */
  while(Send_AT_Command("AT\r\n", "OK", 1000) != HAL_OK) {
      HAL_Delay(200);
  }

  OLED_Clear();
  OLED_PrintString("MODEM CONNECTED", 0, 10);

  Init_GPRS_Stack();

    while (1)
    {
        /* 1. Network Status Check */
        if (Send_AT_Command("AT+CREG?\r\n", "OK", 1000) == HAL_OK) {
            if (strstr((char*)rx_buffer, ",1") || strstr((char*)rx_buffer, ",5")) {
                strcpy(dev_status, "CONNECTED");
            } else {
                strcpy(dev_status, "SEARCHING");
            }
        }

        /* 2. Check IP Address Status (If GPRS drops, reconnect) */
        if (Send_AT_Command("AT+SAPBR=2,1\r\n", "0.0.0.0", 1000) == HAL_OK) {
             Init_GPRS_Stack();
        }

        /* 3. Fetch GPS Coordinates (LBS) */
        Flush_Buffer();
        OLED_PrintString("PINGING TOWER... ", 2, 0); // Visual feedback
        HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CIPGSMLOC=1,1\r\n", 18, 100);

        uint32_t tick = HAL_GetTick();
        uint8_t loc_success = 0;
        uint8_t cmd_error = 0;

        // WAIT 35 SECONDS. Network APIs are slow.
        while ((HAL_GetTick() - tick) < 35000) {
            if (strstr((char*)rx_buffer, "+CIPGSMLOC:") != NULL) {
                loc_success = 1;
                break;
            }
            if (strstr((char*)rx_buffer, "ERROR") != NULL) {
                cmd_error = 1;
                break;
            }
        }

        if (loc_success) {
            char *loc_ptr = strstr((char*)rx_buffer, "+CIPGSMLOC:");
            int res_code = -1;
            char temp_date[12] = {0};
            char temp_time[12] = {0};

            if (loc_ptr != NULL) {
                int parsed = sscanf(loc_ptr, "+CIPGSMLOC: %d,%15[^,],%15[^,],%11[^,],%11[^\r\n]",
                           &res_code, lon_str, lat_str, temp_date, temp_time);

                // If it parsed a response code, but it's not 0 (Success)
                if (parsed >= 1 && res_code != 0) {
                    sprintf(lat_str, "ERR:%d", res_code);
                    sprintf(lon_str, "ERR:%d", res_code);
                }
            }
        } else {
            if (cmd_error) {
                strcpy(lat_str, "CMD_ERR"); // GPRS/APN Failed
                strcpy(lon_str, "CMD_ERR");
            } else {
                strcpy(lat_str, "TIMEOUT"); // No reply at all after 35s
                strcpy(lon_str, "TIMEOUT");
            }
        }

        /* 4. Minimal UI Update */
        OLED_Clear();
        sprintf(screen_buf, "STATUS: %s", dev_status);
        OLED_PrintString(screen_buf, 0, 0);

        sprintf(screen_buf, "LAT: %s", lat_str);
        OLED_PrintString(screen_buf, 3, 0);

        sprintf(screen_buf, "LON: %s", lon_str);
        OLED_PrintString(screen_buf, 5, 0);

        /* 5. Loop Delay */
        HAL_Delay(10000);
    }
}

/** HAL configurations generated by CubeMX remain completely standard below **/

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
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
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
