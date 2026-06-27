/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with SIM900A and I2C OLED Integration
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED_ADDR (0x3C << 1) // Standard SSD1306 I2C address
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1; // I2C Handle for PB8/PB9 communication

/* USER CODE BEGIN PV */
uint8_t tx_buffer[] = "AT\r\n";
uint8_t rx_buffer[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void); // I2C1 Prototype
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Alphanumeric font array mapping printable characters from ASCII 0x20 (Space) to 0x5A ('Z') */
const uint8_t SSD1306_Font[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x03, 0x00, 0x03, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x24, 0x24, 0x24, 0x24, 0x24}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}  // Z
};

void OLED_WriteCmd(uint8_t cmd) {
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x00, 1, &cmd, 1, HAL_MAX_DELAY);
}

void OLED_Init(void) {
    HAL_Delay(100);
    OLED_WriteCmd(0xAE);
    OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xB0);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0xA1);
    OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xAF);
}

void OLED_Clear(void) {
    uint8_t buffer[128] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        OLED_WriteCmd(0xB0 + i);
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);
        HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x40, 1, buffer, 128, HAL_MAX_DELAY);
    }
}

void OLED_PrintChar(char c, uint8_t page, uint8_t col) {
    if (c >= 'a' && c <= 'z') {
        c -= 32;
    }
    if (c < 32 || c > 90) return;

    OLED_WriteCmd(0xB0 + page);
    OLED_WriteCmd(0x00 + (col & 0x0F));
    OLED_WriteCmd(0x10 + ((col >> 4) & 0x0F));

    uint8_t dataData[5];
    for(int i = 0; i < 5; i++) {
        dataData[i] = SSD1306_Font[c - 32][i];
    }
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x40, 1, dataData, 5, HAL_MAX_DELAY);
}

void OLED_PrintStr(char* str, uint8_t page, uint8_t start_col) {
    while(*str) {
        OLED_PrintChar(*str, page, start_col);
        start_col += 6;
        str++;
    }
}

void SIM900A_SendCommand(char *command) {
    HAL_UART_Transmit(&huart1, (uint8_t *)command, strlen(command), 1000);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();
  OLED_PrintStr("SYSTEM BOOTING...", 0, 16);

    HAL_Delay(5000);
    OLED_Clear();

    // 1. Ping the module
    SIM900A_SendCommand("AT\r\n");
    HAL_Delay(5000);

    // 2. NEW FUNCTIONALITY: Enable automatic cellular local network time synchronization (IST)
    SIM900A_SendCommand("AT+CLTS=1\r\n");
    HAL_Delay(5000);

    // 3. Put the module into SMS Text Mode
    SIM900A_SendCommand("AT+CMGF=1\r\n");
    HAL_Delay(5000);

    // 4. Input the target phone number.
    SIM900A_SendCommand("AT+CMGS=\"9370639073\"\r\n");
    HAL_Delay(5000);

    // 5. Write message
    SIM900A_SendCommand("Alert from STM32! Approx Location: Lat 25.15, Long 82.30");
    HAL_Delay(5000);

    // 6. Send CTRL+Z (\x1A)
    char ctrl_z[2] = {0x1A, 0x00};
    SIM900A_SendCommand(ctrl_z);

    HAL_Delay(5000);
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        char at_command[] = "AT\r\n";
        uint8_t rx_buffer[20] = {0};

        // Send the "AT" command via USART1
        HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), 100);
        HAL_UART_Receive(&huart1, rx_buffer, sizeof(rx_buffer), 1000);

        // Check if the response contains "OK"
        if (strstr((char*)rx_buffer, "OK") != NULL)
        {
            OLED_PrintStr("AT acknowledged", 3, 16);

            // NEW FUNCTIONALITY: Request current RTC time from cellular network
            char cclk_command[] = "AT+CCLK?\r\n";
            uint8_t rx_time_buffer[64] = {0};

            HAL_UART_Transmit(&huart1, (uint8_t*)cclk_command, strlen(cclk_command), 100);
            HAL_UART_Receive(&huart1, rx_time_buffer, sizeof(rx_time_buffer) - 1, 1000);

            // Parse response string looking for structural marker: +CCLK: "yy/mm/dd,hh:mm:ss+22"
            char *time_ptr = strstr((char*)rx_time_buffer, "+CCLK: \"");
            if (time_ptr != NULL)
            {
                char *comma_ptr = strchr(time_ptr, ',');
                if (comma_ptr != NULL)
                {
                    char time_str[12] = {0};
                    strncpy(time_str, comma_ptr + 1, 8); // Extract "hh:mm:ss"

                    // Display the time string directly below on Page 5
                    OLED_PrintStr("TIME: ", 5, 16);
                    OLED_PrintStr(time_str, 5, 52);                }
            }
            else
            {
                OLED_PrintStr("SYNCING TIME...", 5, 16);
            }

            // Communication successful! Blink the LED on PB15 (Takes 800ms)
            int i;
            for(i = 0; i < 10; i++){
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
                HAL_Delay(80);
            }

            HAL_Delay(1000);  // 800ms (blink) + 200ms = Exactly 1 second total display time
            OLED_Clear();    // Clear screen after 1 second
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            OLED_PrintStr("PINGING MODEM...", 3, 16);

            HAL_Delay(2000);  // Standard fallback clears text window screen after 500ms
            OLED_Clear();
        }

        // Wait 4 seconds to balance out the execution window (5-second structural macro interval)
        HAL_Delay(4000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30F03D5B;
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure Pin PB15 explicitly as Output Push-Pull for LED */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
