/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Основной файл программы
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
/*
 * main.c — точка входа проекта Plata.
 *
 * Здесь выполняется:
 *   - инициализация HAL и тактирования 72 МГц (с fallback на HSE/HSI);
 *   - инициализация I2C1/I2C2, USART1/USART2 и GPIO;
 *   - настройка RS485 для внешнего терминала;
 *   - запуск приложения App_Init() и главный цикл App_Loop().
 *
 * Файл частично сгенерирован CubeMX. Пользовательский код находится
 * в секциях USER CODE BEGIN/END — при повторной генерации они сохраняются.
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "plata.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* Настройка тактирования:
 *   1) HSE 8 МГц + PLL x9 = 72 МГц (основной вариант);
 *   2) HSE 8 МГц напрямую;
 *   3) HSI 8 МГц напрямую.
 * Fallback нужен для клонов STM32F103 с нестабильным кварцем. */
void SystemClock_Config(void);
/* Инициализация GPIO: светодиод PC13 и пины DE/RE обоих RS485. */
static void MX_GPIO_Init(void);
/* Инициализация I2C1 (NAU7802 №1), 100 кГц. */
static void MX_I2C1_Init(void);
/* Инициализация I2C2 (NAU7802 №2), 100 кГц. */
static void MX_I2C2_Init(void);
/* Инициализация USART1: внешний RS485, 115200 8N1. */
static void MX_USART1_UART_Init(void);
/* Инициализация USART2: MKS SERVO42D, 38400 8N1. */
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Точка входа приложения.
  * @retval int
  */
/* Главная функция: инициализация периферии и вечный цикл приложения. */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Сброс периферии, инициализация Flash-интерфейса и SysTick. */
  /* HAL_Init(): сброс периферии, настройка SysTick и Flash-интерфейса. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Настройка системного тактирования. */
  /* 72 МГц через HSE+PLL, с fallback на HSE/HSI. */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Инициализация всех настроенных периферийных блоков. */
  /* Сначала GPIO: светодиод и DE/RE обоих RS485. */
  MX_GPIO_Init();
  /* I2C1 — первый NAU7802. */
  MX_I2C1_Init();
  /* I2C2 — второй NAU7802 (сейчас не подключён). */
  MX_I2C2_Init();
  /* USART1 — внешний RS485/CLI, 115200. */
  MX_USART1_UART_Init();
  /* USART2 — MKS SERVO42D, 38400. */
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Запоминаем внешний UART и переводим его DE/RE в приём. */
  RS485_ExtInit(&huart1);
  /* Инициализация прикладного слоя: датчики, мотор, демо, CLI. */
  App_Init(&hi2c1, &hi2c2, &huart1, &huart2);
  /* USER CODE END 2 */

  /* Бесконечный цикл приложения. */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Внутри: JOG-автостоп, демо, потоки, CLI, LED. */
    App_Loop();
  }
  /* USER CODE END 3 */
}

/**
  * @brief Настройка системного тактирования
  * @retval Нет
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*
   * 1. Основной вариант: HSE 8 МГц + PLL x9 = 72 МГц.
   * 2. Если HSE/PLL не запустились: HSE 8 МГц напрямую.
   * 3. Если HSE нет: HSI 8 МГц напрямую.
   *
   * Fallback важен для клонов F103C8T6 с нестабильным кварцем.
   */
  /* Основной вариант: HSE 8 МГц, PLL ×9 = 72 МГц. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

  /* Если PLL запустился — настраиваем шины и Flash latency.
   * APB1 = 36 МГц (максимум для F103), APB2 = 72 МГц. */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK)
  {
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) == HAL_OK)
    {
      return;
    }
  }

  /* Если PLL не запустился, пробуем HSE 8 МГц без PLL. */
  /* Fallback 1: HSE 8 МГц напрямую */
  RCC_OscInitStruct = (RCC_OscInitTypeDef){0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK)
  {
    RCC_ClkInitStruct = (RCC_ClkInitTypeDef){0};
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) == HAL_OK)
    {
      return;
    }
  }

  /* Последний вариант — внутренний HSI 8 МГц. */
  /* Fallback 2: HSI 8 МГц напрямую */
  RCC_OscInitStruct = (RCC_OscInitTypeDef){0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct = (RCC_ClkInitTypeDef){0};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  * @brief Инициализация I2C1
  * @param Нет
  * @retval Нет
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief Инициализация I2C2
  * @param Нет
  * @retval Нет
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief Инициализация USART1
  * @param Нет
  * @retval Нет
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief Инициализация USART2
  * @param Нет
  * @retval Нет
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief Инициализация GPIO
  * @param Нет
  * @retval Нет
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* Включение тактирования портов GPIO. */
  /* Включаем тактирование портов, к которым подключены LED и DE. */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Начальный уровень выходов GPIO. */
  /* PC13 активен низким уровнем: HIGH = светодиод погашен. */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

  /* Начальный уровень выходов GPIO. */
  /* DE/RE = 0 — оба RS485 в режиме приёма. */
  HAL_GPIO_WritePin(GPIOB, EXT_RS485_DE_Pin|MKS_RS485_DE_Pin, GPIO_PIN_RESET);

  /* Светодиод статуса PC13 — выход push-pull. */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /* Пины DE/RE внешнего и моторного RS485 — выходы push-pull. */
  GPIO_InitStruct.Pin = EXT_RS485_DE_Pin|MKS_RS485_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static UART_HandleTypeDef *ext_uart = NULL;

/* Короткая задержка перед поднятием DE, чтобы передатчик успел
 * переключиться. Реализована через NOP, без блокирующего HAL_Delay. */
static void rs485_de_setup_delay(void)
{
  volatile uint32_t i;

  for (i = 0U; i < 2000U; i++)
  {
    __NOP();
  }
}

/* Запоминает UART внешнего RS485 и переводит DE/RE в приём. */
void RS485_ExtInit(UART_HandleTypeDef *huart)
{
  ext_uart = huart;
  HAL_GPIO_WritePin(EXT_RS485_DE_GPIO_Port, EXT_RS485_DE_Pin, GPIO_PIN_RESET);
}

/* Передача по RS485 с управлением DE/RE:
 *   - для внешнего USART1 на время передачи отключается RX-прерывание,
 *     чтобы приёмник SP3485 не ловил собственный сигнал;
 *   - DE поднимается, данные отправляются, ждём TC, DE опускается. */
HAL_StatusTypeDef RS485_Transmit(UART_HandleTypeDef *huart,
                                 GPIO_TypeDef *de_port,
                                 uint16_t de_pin,
                                 const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout)
{
  HAL_StatusTypeDef status;
  uint32_t start;
  /* Для внешнего USART1 нужно на время передачи отключить RX-прерывание:
   * приёмник SP3485 в режиме передачи отключён, и PA10 может «висеть». */
  uint8_t is_ext = (huart != NULL && huart->Instance == USART1) ? 1U : 0U;

  if (huart == NULL || data == NULL || len == 0U)
  {
    return HAL_ERROR;
  }

  /*
   * Внешний RS485: на время передачи отключаем RX-прерывание USART1.
   * Приёмник SP3485 в режиме передачи отключён, поэтому PA10 может «висеть».
   */
  /* Отключаем прерывание приёма только для внешнего RS485. */
  if (is_ext)
  {
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }

  /* DE=1 — включаем передатчик RS485. */
  HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_SET);
  rs485_de_setup_delay(); /* короткая задержка DE, без блокирующего 1 мс */

  /* HAL_UART_Transmit блокирующий: кладёт байты в DR и ждёт отправки. */
  status = HAL_UART_Transmit(huart, (uint8_t *)data, len, timeout);

  start = HAL_GetTick();
  /* Ждём TC (transmission complete), чтобы опустить DE после
   * последнего бита, а не после записи в DR. */
  while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
  {
    if ((HAL_GetTick() - start) > timeout)
    {
      status = HAL_TIMEOUT;
      break;
    }
  }

  /* DE=0 — снова приём. */
  HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_RESET);

  /* Возвращаем прерывание приёма внешнего RS485. */
  if (is_ext)
  {
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }

  return status;
}

/* Отправка данных во внешний RS485 (USART1). */
HAL_StatusTypeDef RS485_ExtSend(const uint8_t *data, uint16_t len)
{
  if (ext_uart == NULL)
  {
    return HAL_ERROR;
  }

  return RS485_Transmit(ext_uart,
                        EXT_RS485_DE_GPIO_Port,
                        EXT_RS485_DE_Pin,
                        data,
                        len,
                        100U);
}

/* Перенаправление newlib printf/puts во внешний RS485. */
/* Перенаправление printf/puts в внешний RS485 (newlib). */
/* newlib вызывает _write для printf/puts. Перенаправляем всё
 * в RS485_ExtSend, чтобы printf уходил в терминал. */
int _write(int file, char *ptr, int len)
{
  (void)file;

  if (ptr == NULL || len <= 0)
  {
    return 0;
  }

  /* Если RS485 не принял данные — возвращаем -1, как требует newlib. */
  if (RS485_ExtSend((const uint8_t *)ptr, (uint16_t)len) != HAL_OK)
  {
    return -1;
  }

  return len;
}
/* USER CODE END 4 */

/**
  * @brief  Вызывается при возникновении ошибки.
  * @retval Нет
  */
/* Аварийный обработчик: зажигает LED и останавливает выполнение. */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Светодиод PC13 активен низким уровнем: включить и остаться здесь */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Сообщает имя файла и номер строки, где произошла ошибка assert_param.
  * @param  file: указатель на имя файла
  * @param  line: номер строки с ошибкой assert_param
  * @retval Нет
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* Здесь можно выводить имя файла и номер строки при ошибке assert. */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
