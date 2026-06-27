/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Standalone NEO-6M GPS to 16x2 I2C LCD Display (VCP CSV Logger)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
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

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* Global variables marked 'volatile' so they update perfectly in Live Expressions */
volatile float latitude = 0.0f;
volatile float longitude = 0.0f;

/* Raw strings just in case you want to see the exact NMEA output */
char raw_lat[20] = "Waiting...";
char raw_lon[20] = "Waiting...";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int fetch_and_parse_gps(void);
float nmea_to_decimal(const char *nmea, char dir);
void float_to_string(float val, char* buf);
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
  MX_UART4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Print standard CSV Header required by GPS Visualizer out of the serial interface */
  printf("latitude,longitude\r\n");

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Fetch the GPS data and look for a valid return token (1 = Success) */
    if (fetch_and_parse_gps() == 1)
    {
        /* Buffers to hold our integer-converted string values */
        char lat_str[20];
        char lon_str[20];

        /* Convert float to string using standard integer math (Bypasses CubeIDE %f warning) */
        float_to_string((float)latitude, lat_str);
        float_to_string((float)longitude, lon_str);

        /* Transmit data automatically across the serial logging stream using string formatting */
        printf("%s,%s\r\n", lat_str, lon_str);

        /* Toggle LED ONLY when valid data is received to indicate active live parsing */
        BSP_LED_Toggle(LED_GREEN);
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
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
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/**
  * @brief Redirects standard standard printf outputs to the STM32 VCP (Virtual COM Port)
  */
int _write(int file, char *ptr, int len)
{
    extern UART_HandleTypeDef hcom_uart[];
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/**
  * @brief Helper function to format float to string manually (bypasses _printf_float requirement)
  */
void float_to_string(float val, char* buf) {
    int int_part = (int)val;
    float remainder = val - (float)int_part;

    if (remainder < 0.0f) {
        remainder = -remainder;
    }

    /* Calculate 6 decimal places */
    int frac_part = (int)(remainder * 1000000.0f);

    /* Handle edge case where value is negative but integer part is zero (e.g. -0.12345) */
    if (val < 0.0f && int_part == 0) {
        sprintf(buf, "-0.%06d", frac_part);
    } else {
        sprintf(buf, "%d.%06d", int_part, frac_part);
    }
}

/**
  * @brief Converts raw NMEA coordinates (DDMM.MMMMM) into Decimal Degrees (DD.DDDDD)
  */
float nmea_to_decimal(const char *nmea, char dir) {
    if (strlen(nmea) < 4) return 0.0f;

    char deg_str[4] = {0};
    /* Longitude uses 3 digits for degrees (DDD), Latitude uses 2 (DD) */
    int deg_len = (dir == 'E' || dir == 'W') ? 3 : 2;

    strncpy(deg_str, nmea, deg_len);
    const char *min_str = nmea + deg_len;

    float degrees = atof(deg_str);
    float minutes = atof(min_str);

    /* Convert formula: DD + (MM.MMMM / 60) */
    float decimal = degrees + (minutes / 60.0f);

    /* Southern and Western hemispheres are negative coordinates */
    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

/**
  * @brief Polls UART4 for the NEO-6M $GPGGA string and parses out the coordinates.
  * @retval int 1 if valid new coordinates parsed successfully, 0 if timeout/no fix.
  */
int fetch_and_parse_gps(void) {
    char rx_buf[256] = {0};
    uint16_t rx_idx = 0;
    uint8_t rx_byte;
    uint32_t start_tick = HAL_GetTick();

    /* Auto-Recovery Check: Clear UART Errors */
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(&huart4, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(&huart4, UART_FLAG_FE)) {
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF);
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_NEF);
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_FEF);
        __HAL_UART_FLUSH_DRREGISTER(&huart4);
    }

    /* Listen for up to 1.5 seconds to ensure we catch a full 1Hz GPS broadcast */
    while ((HAL_GetTick() - start_tick) < 1500) {

        /* Continuous check inside loop to clear overruns */
        if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF);
        }

        if (HAL_UART_Receive(&huart4, &rx_byte, 1, 10) == HAL_OK) {
            if (rx_idx < 255) {
                rx_buf[rx_idx++] = rx_byte;
                rx_buf[rx_idx] = '\0';
            }

            /* Once a full NMEA sentence is captured */
            if (rx_byte == '\n') {
                char *gga = strstr(rx_buf, "$GPGGA,");
                if (gga != NULL) {
                    char *time_end = strchr(gga + 7, ',');
                    if (time_end) {
                        char *lat_start = time_end + 1;
                        char *lat_end = strchr(lat_start, ',');

                        /* Verify GPS has a fix (data exists between commas) */
                        if (lat_end && (lat_end - lat_start > 1)) {
                            char *ns_start = lat_end + 1;
                            char *ns_end = strchr(ns_start, ',');

                            /* Safe pointer checks */
                            if (ns_end) {
                                char *lon_start = ns_end + 1;
                                char *lon_end = strchr(lon_start, ',');

                                if (lon_end && (lon_end - lon_start > 1)) {
                                    char *ew_start = lon_end + 1;

                                    /* Extract raw NMEA strings */
                                    strncpy(raw_lat, lat_start, lat_end - lat_start);
                                    raw_lat[lat_end - lat_start] = '\0';

                                    strncpy(raw_lon, lon_start, lon_end - lon_start);
                                    raw_lon[lon_end - lon_start] = '\0';

                                    /* Update global volatile variables */
                                    latitude = nmea_to_decimal(raw_lat, ns_start[0]);
                                    longitude = nmea_to_decimal(raw_lon, ew_start[0]);

                                    return 1; /* Success! Return 1 */
                                }
                            }
                        }
                    }
                }
                /* Reset buffer to catch the next line if this one wasn't valid */
                rx_idx = 0;
                rx_buf[0] = '\0';
            }
        }
    }
    return 0; /* Timeout or no fix */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
