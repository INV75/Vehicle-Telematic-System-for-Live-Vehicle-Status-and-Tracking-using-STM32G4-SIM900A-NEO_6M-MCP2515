/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SLCAN Bridge for CANHacker - Hardware Active Edition
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MCP2515.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t ext;
} CanMessage;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLCAN_BUF_SIZE 64

// ===========================================================================
// HARDWARE ASSIGNMENTS: Verify these match your physical wire setup!
// ===========================================================================
#define MCP_CRYSTAL_FREQ        8        // Set to 8 for 8MHz crystal, or 16 for 16MHz
#define MCP2515_CS_PORT         GPIOB    // Port where your CS/NSS wire is connected
#define MCP2515_CS_PIN          GPIO_PIN_6 // Pin where your CS/NSS wire is connected

// Bitrate Selection Constants
#define CAN_250KBPS   0
#define CAN_500KBPS   1
#define CAN_1000KBPS  2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- SLCAN / Lawicel State Variables ---
uint8_t slcan_rx_byte;
char slcan_buffer[SLCAN_BUF_SIZE];
uint8_t slcan_index = 0;
volatile uint8_t slcan_cmd_ready = 0;

// Defaulted to 1 for instant vehicle bus sniffing during direct debug sessions
uint8_t can_channel_open = 1;

// Debugging Flags - Watch these inside your Live Expressions panel
volatile uint8_t mcp2515_spi_verified = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
void Process_SLCAN_Command(void);
void Send_Serial_String(const char* str);

// High-Level Driver Wrapper Prototypes
void MCP2515_SetBitrate(uint8_t bitrate, uint8_t crystal_mhz);
void MCP2515_SetListenOnlyMode(void);
uint8_t MCP2515_CheckReceive(void);
void MCP2515_ReadMessage(CanMessage *msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// UART Interrupt Callback to catch incoming PC commands
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (slcan_rx_byte == '\r') {
            slcan_buffer[slcan_index] = '\0';
            slcan_cmd_ready = 1;
        } else {
            if (slcan_index < SLCAN_BUF_SIZE - 1) {
                slcan_buffer[slcan_index++] = slcan_rx_byte;
            }
        }
        HAL_UART_Receive_IT(&huart2, &slcan_rx_byte, 1);
    }
}

// Lawicel Protocol Parser
void Process_SLCAN_Command(void) {
    char cmd = slcan_buffer[0];

    switch (cmd) {
        case 'S': // Setup Bitrate manually from host software
            if (slcan_buffer[1] == '6') { MCP2515_SetBitrate(CAN_500KBPS, MCP_CRYSTAL_FREQ); }
            else if (slcan_buffer[1] == '5') { MCP2515_SetBitrate(CAN_250KBPS, MCP_CRYSTAL_FREQ); }
            else if (slcan_buffer[1] == '8') { MCP2515_SetBitrate(CAN_1000KBPS, MCP_CRYSTAL_FREQ); }
            Send_Serial_String("\r");
            break;

        case 'O': // Open CAN channel
            MCP2515_SetNormalMode();
            can_channel_open = 1;
            Send_Serial_String("\r");
            break;

        case 'C': // Close CAN channel
            MCP2515_SetListenOnlyMode();
            can_channel_open = 0;
            Send_Serial_String("\r");
            break;

        case 'v': // Get Version
            Send_Serial_String("V1013\r");
            break;

        case 'V': // Get Serial Number
            Send_Serial_String("N1234\r");
            break;

        default:
            Send_Serial_String("\a");
            break;
    }
    slcan_index = 0;
    slcan_cmd_ready = 0;
}

void Send_Serial_String(const char* str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 10);
}

// --- Implementation of High-Level Driver Logic Mapping to Hardware Registers ---

void MCP2515_SetBitrate(uint8_t bitrate, uint8_t crystal_mhz) {
    // Force device into Configuration Mode (REQOP bits = 100 in CANCTRL register 0x0F)
    MCP2515_BitModify(0x0F, 0xE0, 0x80);

    if (crystal_mhz == 8) {
        if (bitrate == CAN_500KBPS) {
            MCP2515_WriteByte(0x2A, 0x00); // CNF1
            MCP2515_WriteByte(0x29, 0x91); // CNF2
            MCP2515_WriteByte(0x28, 0x01); // CNF3
        } else if (bitrate == CAN_250KBPS) {
            MCP2515_WriteByte(0x2A, 0x01); // CNF1
            MCP2515_WriteByte(0x29, 0x91); // CNF2
            MCP2515_WriteByte(0x28, 0x01); // CNF3
        } else if (bitrate == CAN_1000KBPS) {
            MCP2515_WriteByte(0x2A, 0x00); // CNF1
            MCP2515_WriteByte(0x29, 0x80); // CNF2
            MCP2515_WriteByte(0x28, 0x00); // CNF3
        }
    }
    else if (crystal_mhz == 16) {
        if (bitrate == CAN_500KBPS) {
            MCP2515_WriteByte(0x2A, 0x00); // CNF1
            MCP2515_WriteByte(0x29, 0x9E); // CNF2
            MCP2515_WriteByte(0x28, 0x03); // CNF3
        } else if (bitrate == CAN_250KBPS) {
            MCP2515_WriteByte(0x2A, 0x01); // CNF1
            MCP2515_WriteByte(0x29, 0x9E); // CNF2
            MCP2515_WriteByte(0x28, 0x03); // CNF3
        } else if (bitrate == CAN_1000KBPS) {
            MCP2515_WriteByte(0x2A, 0x00); // CNF1
            MCP2515_WriteByte(0x29, 0x91); // CNF2
            MCP2515_WriteByte(0x28, 0x01); // CNF3
        }
    }
}

void MCP2515_SetListenOnlyMode(void) {
    MCP2515_BitModify(0x0F, 0xE0, 0x60);
}

uint8_t MCP2515_CheckReceive(void) {
    uint8_t status = MCP2515_ReadStatus();
    return (status & 0x03);
}

void MCP2515_ReadMessage(CanMessage *msg) {
    uint8_t status = MCP2515_ReadStatus();
    uint8_t baseReg = 0x61;
    uint8_t intfBit = 0x01;

    if (!(status & 0x01) && (status & 0x02)) {
        baseReg = 0x71;
        intfBit = 0x02;
    }

    uint8_t idh = MCP2515_ReadByte(baseReg);
    uint8_t idl = MCP2515_ReadByte(baseReg + 1);
    uint8_t e8  = MCP2515_ReadByte(baseReg + 2);
    uint8_t e0  = MCP2515_ReadByte(baseReg + 3);
    uint8_t dlc = MCP2515_ReadByte(baseReg + 4);

    msg->dlc = dlc & 0x0F;

    if (idl & 0x08) {
        msg->ext = 1;
        msg->id = ((uint32_t)idh << 21) |
                  ((uint32_t)(idl & 0xE0) << 13) |
                  ((uint32_t)(idl & 0x03) << 16) |
                  ((uint32_t)e8 << 8) | e0;
    } else {
        msg->ext = 0;
        msg->id = ((uint32_t)idh << 3) | ((idl & 0xE0) >> 5);
    }

    for (int i = 0; i < msg->dlc; i++) {
        msg->data[i] = MCP2515_ReadByte(baseReg + 5 + i);
    }

    MCP2515_BitModify(0x2C, intfBit, 0x00);
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
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  // 1. Initialize lower level hardware layers
  MCP2515_Initialize();

  // 2. SPI Integrity Validation Test
  // Read back the CANSTAT control register (0x0E). In default config state, it must reflect 0x80.
  uint8_t check_reg = MCP2515_ReadByte(0x0E);
  if ((check_reg & 0xE0) == 0x80) {
      mcp2515_spi_verified = 1; // SPI interface functional!
  } else {
      mcp2515_spi_verified = 0; // Hardware/wiring fault detected between STM32 and MCP2515
  }

  // 3. Force auto-configuration for standard 500kbps vehicle sniffing immediately
  MCP2515_SetBitrate(CAN_500KBPS, MCP_CRYSTAL_FREQ);
  MCP2515_SetNormalMode();

  // 4. Arm UART listening stream
  HAL_UART_Receive_IT(&huart2, &slcan_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Parse data frame inputs from serial connection
      if (slcan_cmd_ready) {
          Process_SLCAN_Command();
      }

      // Live processing loop for CAN bus data frames
      if (can_channel_open && MCP2515_CheckReceive()) {
          CanMessage rx_frame;
          MCP2515_ReadMessage(&rx_frame);

          char tx_buf[64] = {0};
          char hex_byte[3] = {0};

          if (rx_frame.ext) {
              sprintf(tx_buf, "T%08lX%d", rx_frame.id, rx_frame.dlc);
          } else {
              sprintf(tx_buf, "t%03lX%d", rx_frame.id, rx_frame.dlc);
          }

          for (int i = 0; i < rx_frame.dlc; i++) {
              sprintf(hex_byte, "%02X", rx_frame.data[i]);
              strcat(tx_buf, hex_byte);
          }

          strcat(tx_buf, "\r");
          Send_Serial_String(tx_buf);
      }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;   // SPI Mode 0
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;        // SPI Mode 0
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // Safe speed scaling for 170MHz clock
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // FIX APPLIED HERE
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level for CS */
  HAL_GPIO_WritePin(MCP2515_CS_PORT, MCP2515_CS_PIN, GPIO_PIN_SET);

  /*Configure GPIO pin : CS Pin Configuration */
  GPIO_InitStruct.Pin = MCP2515_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MCP2515_CS_PORT, &GPIO_InitStruct);

  /**SPI1 GPIO Configuration
  PA5     ------> SPI1_SCK
  PA6     ------> SPI1_MISO
  PA7     ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**USART2 GPIO Configuration
  PA2     ------> USART2_TX
  PA3     ------> USART2_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */
