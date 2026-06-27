/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
#define OLED_ADDR (0x3C << 1)

// Global RAM frame buffer for the 128x64 display (8 pages of 128 bytes)
uint8_t OLED_Buffer[8][128];

// Complete 5x7 ASCII Font Table (Chars 32 to 95)
const uint8_t SSD1306_Font[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5f, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7f, 0x14, 0x7f, 0x14}, // #
    {0x24, 0x2a, 0x7f, 0x2a, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1c, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1c, 0x00}, // )
    {0x14, 0x08, 0x3e, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3e, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 0
    {0x00, 0x42, 0x7f, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4b, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7f, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1e}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x24, 0x24, 0x24, 0x24, 0x24}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3e}, // @
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, // A
    {0x7f, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3e, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7f, 0x41, 0x41, 0x22, 0x1c}, // D
    {0x7f, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7f, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3e, 0x41, 0x49, 0x49, 0x7a}, // G
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, // H
    {0x00, 0x41, 0x7f, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3f, 0x01}, // J
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7f, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // M
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, // N
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, // O
    {0x7f, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3e, 0x41, 0x51, 0x21, 0x5e}, // Q
    {0x7f, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7f, 0x01, 0x01}, // T
    {0x3f, 0x40, 0x40, 0x40, 0x3f}, // U
    {0x1f, 0x20, 0x40, 0x20, 0x1f}, // V
    {0x3f, 0x40, 0x38, 0x40, 0x3f}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

void oled_cmd(uint8_t cmd);
void oled_data(uint8_t data);
void oled_init(void);
void oled_clear_buffer(void);
void oled_update(void);
uint32_t oled_buffer_write_str(uint8_t page, int16_t col_offset, char* str);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  oled_init();
  oled_clear_buffer();
  oled_update();
  /* USER CODE END 2 */

  /* Initialize standard Nucleo components */
  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  BSP_COM_Init(COM1, &BspCOMInit);

  // Combine the strings into one massive looping ribbon ticker
  char banner_text[] = "   No mo half measures, Walter.  * STAY OUT OF MY TERRITORY!  * ";

  // Calculate total length of the banner message in terms of screen columns
  int16_t text_pixel_width = strlen(banner_text) * 6;

  // Start the text completely hidden off the right edge of the screen (128)
  int16_t scroll_position = 128;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. Wipe the current buffer frame clean
      oled_clear_buffer();

      // 2. Write text string directly to the virtual buffer layer at our sliding coordinate
      // Placed on page 3 to keep it perfectly centered on the 64-pixel high display
      oled_buffer_write_str(3, scroll_position, banner_text);

      // 3. Push local RAM buffer changes out to physical GDDRAM hardware via I2C
      oled_update();

      // 4. Shift coordinate one pixel to the left
      scroll_position--;

      // Reset position once text has completely crawled past the left edge (-text_pixel_width)
      if (scroll_position < -text_pixel_width) {
          scroll_position = 128;
      }

      // 5. Control speed: lower values crawl fast, higher values slide slow
      HAL_Delay(15);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
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
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40621236;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void oled_cmd(uint8_t cmd) {
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x00, 1, &cmd, 1, HAL_MAX_DELAY);
}

void oled_data(uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x40, 1, &data, 1, HAL_MAX_DELAY);
}

void oled_init(void) {
    HAL_Delay(100);

    oled_cmd(0xAE); // Display OFF
    oled_cmd(0xD5); // Set Display Clock Divide Ratio
    oled_cmd(0x80);
    oled_cmd(0xA8); // Set Multiplex Ratio
    oled_cmd(0x3F); // 64 rows
    oled_cmd(0xD3); // Set Display Offset
    oled_cmd(0x00);
    oled_cmd(0x40); // Set Display Start Line to 0
    oled_cmd(0x8D); // Charge Pump Setup
    oled_cmd(0x14); // Enable Charge Pump
    oled_cmd(0x20); // Set Memory Addressing Mode
    oled_cmd(0x00); // Horizontal Addressing Mode for fast frame buffering updates
    oled_cmd(0xA1); // Segment Re-map
    oled_cmd(0xC8); // COM Output Scan Direction
    oled_cmd(0xDA); // COM Pins Hardware Configuration
    oled_cmd(0x12);
    oled_cmd(0x81); // Contrast Control
    oled_cmd(0xCF);
    oled_cmd(0xD9); // Set Pre-charge Period
    oled_cmd(0xF1);
    oled_cmd(0xDB); // Set VCOMH Deselect Level
    oled_cmd(0x40);
    oled_cmd(0xA4); // Entire Display ON
    oled_cmd(0xA6); // Set Normal Display
    oled_cmd(0xAF); // Display ON
}

// Clears memory layout on the local buffer layer
void oled_clear_buffer(void) {
    memset(OLED_Buffer, 0x00, sizeof(OLED_Buffer));
}

// Dumps internal local buffer straight across to SSD1306 GDDRAM hardware
void oled_update(void) {
    oled_cmd(0x21); // Column Address Setup command
    oled_cmd(0x00); // Start column index 0
    oled_cmd(127);  // End column index 127

    oled_cmd(0x22); // Page Address Setup command
    oled_cmd(0x00); // Start page index 0
    oled_cmd(7);    // End page index 7

    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR, 0x40, 1, (uint8_t*)OLED_Buffer, 1024, HAL_MAX_DELAY);
}

// Prints text directly onto local frame layer mapping buffer with strict screen boundaries
uint32_t oled_buffer_write_str(uint8_t page, int16_t col_offset, char* str) {
    int16_t current_col = col_offset;

    while (*str) {
        char c = *str++;

        if (c >= 'a' && c <= 'z') {
            c -= 32;
        }

        if (c >= 32 && c <= 95) {
            for (uint8_t i = 0; i < 5; i++) {
                if (current_col >= 0 && current_col < 128) {
                    OLED_Buffer[page][current_col] = SSD1306_Font[c - 32][i];
                }
                current_col++;
            }

            if (current_col >= 0 && current_col < 128) {
                OLED_Buffer[page][current_col] = 0x00;
            }
            current_col++;
        }
    }
    return current_col;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
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
