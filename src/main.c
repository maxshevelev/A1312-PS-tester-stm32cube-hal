/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "ssd1306.h"
#include "fonts.h"
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
#define I2C_BUS                 hi2c1 // i2c2 and i2c3 do not work for some reason with HAL
#define I2C_TIMEOUT             20
#define INA219_I2C_ADDR         0x80
#define EMC1403_1_I2C_ADDR      0x98
#define EMC1403_2_I2C_ADDR      0x9A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
  uint16_t value;
  HAL_StatusTypeDef status;
} RESULT_16;

HAL_StatusTypeDef writeRegisgter16(uint8_t deviceAddr, uint8_t regAddr, uint16_t value) {
      uint8_t buf[3] = {regAddr, (uint8_t)(value >> 8), (uint8_t)value};
      return HAL_I2C_Master_Transmit(&I2C_BUS, deviceAddr, buf, 3, I2C_TIMEOUT);
}

RESULT_16 readRegisgter16(uint8_t deviceAddr, uint8_t regAddr) {
      uint8_t buf[2] = {regAddr, 0};
      HAL_StatusTypeDef result;
      result = HAL_I2C_Master_Transmit(&I2C_BUS, deviceAddr, buf, 1, I2C_TIMEOUT);
      if (result != HAL_OK) { return (RESULT_16){0, result}; }
      result = HAL_I2C_Master_Receive(&I2C_BUS, deviceAddr, buf, 2, I2C_TIMEOUT);
      if (result != HAL_OK) { return (RESULT_16){0, result}; }
      uint16_t value = (buf[0] << 8) + buf[1];
      return (RESULT_16){value, result};
}

typedef struct {
  uint8_t value;
  HAL_StatusTypeDef status;
} RESULT_8;

HAL_StatusTypeDef writeRegisgter8(uint8_t deviceAddr, uint8_t regAddr, uint8_t value) {
      uint8_t buf[2] = {regAddr, value};
      return HAL_I2C_Master_Transmit(&I2C_BUS, deviceAddr, buf, 2, I2C_TIMEOUT);
}

RESULT_8 readRegisgter8(uint8_t deviceAddr, uint8_t regAddr) {
      uint8_t buf[1] = {regAddr};
      HAL_StatusTypeDef result;
      result = HAL_I2C_Master_Transmit(&I2C_BUS, deviceAddr, buf, 1, I2C_TIMEOUT);
      if (result != HAL_OK) { return (RESULT_8){0, result}; }
      result = HAL_I2C_Master_Receive(&I2C_BUS, deviceAddr, buf, 1, I2C_TIMEOUT);
      if (result != HAL_OK) { return (RESULT_8){0, result}; }
      return (RESULT_8){buf[0], result};
}

double convertTemperature(uint8_t highByte, uint8_t lowByte) {
  uint16_t w = highByte << 8 | lowByte;
  w = w >> 5;
  return (double)w / 8;
}

char* errMessage(HAL_StatusTypeDef status) {
  switch (status) {
  case HAL_OK: return "OK";
  case HAL_ERROR: return "ERR";
  case HAL_BUSY: return "BUSY";
  case HAL_TIMEOUT: return "TOUT";
  default: return "???";
  }
}

uint8_t probe_EMC1403() {
  // digital filter level 1 on ext diode 1
  if (writeRegisgter8(EMC1403_1_I2C_ADDR, 0x40, 2) == HAL_OK) {
    return EMC1403_1_I2C_ADDR;
  }
  if (writeRegisgter8(EMC1403_2_I2C_ADDR, 0x40, 2) == HAL_OK) {
    return EMC1403_2_I2C_ADDR;
  }
  return 0;
}

typedef struct {
  HAL_StatusTypeDef error;
  double voltage;
  uint8_t cnv_ready;
  uint8_t overload;
  double current;
} PowerData;

typedef struct {
  HAL_StatusTypeDef error;
  double t1;
  double t2;
} TempData;

typedef struct {
  PowerData power;
  TempData temp;
} DisplayData;

void display(DisplayData data) {
    char str[30];

    int voltageRow = 0;
    int currentRow = 22;
    int tempRow = 50;

    SSD1306_Fill(0);
    
    if (data.power.error != HAL_OK) {
      sprintf(str, "PWR: %s", errMessage(data.power.error));
      SSD1306_GotoXY(0, voltageRow);
      SSD1306_Puts(str, &Font_11x18, 1);
    } else {
      if (data.power.overload) {
        sprintf(str, "OL");
      } else {
        sprintf(str, "%.2lfV %c", data.power.voltage, data.power.cnv_ready ? 'R' : ' ');
      }
      SSD1306_GotoXY(0, voltageRow);
      SSD1306_Puts(str, &Font_11x18, 1);

      sprintf(str, "%.2lfA", data.power.current);
      SSD1306_GotoXY(0, currentRow);
      SSD1306_Puts(str, &Font_11x18, 1);
    }

    if (data.temp.error != HAL_OK) {
      sprintf(str, "TEMP: %s", errMessage(data.temp.error));
      SSD1306_GotoXY(0, tempRow);
      SSD1306_Puts(str, &Font_7x10, 1);
    } else {
      sprintf(str, "t1:%.1lf", data.temp.t1);
      SSD1306_GotoXY(0, tempRow);
      SSD1306_Puts(str, &Font_7x10, 1);

      sprintf(str, "t2:%.1lf", data.temp.t2);
      SSD1306_GotoXY(64, tempRow);
      SSD1306_Puts(str, &Font_7x10, 1);
    }

    SSD1306_UpdateScreen();
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
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  SSD1306_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);
    HAL_Delay(500);

    DisplayData data;

    // 0,04096 / (50 / 32768 * 0,032)
    writeRegisgter16(INA219_I2C_ADDR, 0x05, 839); // Current shunt calibration

    {
      RESULT_16 result = readRegisgter16(INA219_I2C_ADDR, 0x02); // read Bus Voltage
      data.power.error = result.status;
      if (result.status == HAL_OK) {
        data.power.error = result.status;
        uint16_t value = result.value;
        data.power.overload = value & 0x1;
        value = value >> 1;
        data.power.cnv_ready = value & 0x1;
        value = value >> 2;
        data.power.voltage = (double)value * 32.0f / 8000.0f;

        result = readRegisgter16(INA219_I2C_ADDR, 0x04); // read Current
        data.power.error = result.status;
        if (result.status == HAL_OK) {
          data.power.current = (double)result.value / 50.0f;
        }
      }
    }

    uint8_t emc1403_addr = probe_EMC1403();
    if (emc1403_addr != 0) {
      RESULT_8 result = readRegisgter8(emc1403_addr, 0x01); // ext diode 1 high byte
      data.temp.error = result.status;
      if (result.status == HAL_OK) {
        uint8_t t1h = result.value;
        result = readRegisgter8(emc1403_addr, 0x10); // ext diode 1 low byte
        uint8_t t1l = result.value;
        data.temp.t1 = convertTemperature(t1h, t1l);

        result = readRegisgter8(emc1403_addr, 0x23); // ext diode 2 high byte
        uint8_t t2h = result.value;
        result = readRegisgter8(emc1403_addr, 0x24); // ext diode 2 low byte
        uint8_t t2l = result.value;
        data.temp.t2 = convertTemperature(t2h, t2l);
      }
    } else {
      data.temp.error = HAL_ERROR;
    }

    display(data);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0x20;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  // GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  // HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_PIN_Pin */
  // GPIO_InitStruct.Pin = LED_PIN_Pin;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(LED_PIN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/