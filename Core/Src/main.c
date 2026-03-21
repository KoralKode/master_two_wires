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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMESLOT 70
#define DWT_CYCCNT   (*((volatile uint32_t*)0xE0001004))
#define DWT_CTRL     (*((volatile uint32_t*)0xE0001000))
#define DATA_BITS  8    // количество бит полезных данных
#define CRC_BITS   5    // количество бит контрольной суммы
// Полином: x⁵ + x² + 1 (0x05 без старшего бита)
#define CRC5_POLY  0x05
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//функция инициализации для задержки в мкс
void DWT_Init(void) {
    DWT_CTRL |= (1 << 0);   // CYCCNTENA
    DWT_CYCCNT = 0;
}

//функция задержки в мкс
void delay_us(uint32_t us) {
    uint32_t start = DWT_CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT_CYCCNT - start) < ticks);
}

//функция отправки одного бита
void send_bit(uint8_t bit){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, bit);
	delay_us(TIMESLOT);//задержка для правильного таймслота
}

//функция приёма одного бита
uint8_t read_bit(){
    // Отпускаем линию (записываем 1) – важно!
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
    delay_us(TIMESLOT/10);               // ждём, пока ведомый выставит уровень
    uint8_t read_data = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    delay_us(TIMESLOT - TIMESLOT/10);    // дожидаемся конца слота
    return read_data;
}

//функция для отправки старт бита здесь 0 не весь таймслот, в его конце линия должна подтянуться к 1
void send_start_bit(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
    delay_us(TIMESLOT - (TIMESLOT/20));   // почти весь слот
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);   // отпускаем линию
    delay_us(TIMESLOT/20);                // дожидаемся конца слота
}



/**
 * @brief Вычисляет CRC-5 для одного байта (MSB first)
 * @param data 8-битное значение
 * @return 5-битная контрольная сумма (0..31)
 */
uint8_t crc5(uint8_t data) {
    uint8_t crc = 0x00;
    for (int i = 7; i >= 0; i--) {
        uint8_t bit = (data >> i) & 1;      // берём бит от старшего к младшему
        uint8_t msb = (crc >> 4) & 1;       // старший бит текущего CRC
        crc = ((crc << 1) | bit) & 0x1F;    // сдвигаем и добавляем новый бит
        if (msb) crc ^= CRC5_POLY;          // если был выдвинут 1 – XOR с полиномом
    }
    return crc;
}

//функция отправки сегмента данных без проверочных битов
void send_segment(uint8_t data, uint8_t crc) {
    // 1. Старт-бит (всегда 0) – мастер притягивает линию на весь слот
	send_start_bit();

    // 2. 8 бит данных (MSB first)
    for (int i = 7; i >= 0; i--) {
        send_bit((data >> i) & 1);
    }

    // 3. 5 бит CRC (MSB first)
    for (int i = 4; i >= 0; i--) {
        send_bit((crc >> i) & 1);
    }
    // Отпускаем линию (записываем 1) – важно!
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);

}

//функция имитирующая (пока) фазу питания
void feed_phase(uint16_t feed_phase_time){
	HAL_Delay(feed_phase_time);
}

//команда пропуска адресса
void ROM_skip(uint8_t * alarm_bit,uint8_t * interrupt_bit,uint8_t *crc_bit){
	uint8_t crc=crc5(0xCC);
	send_segment(0xCC, crc);
	*alarm_bit=read_bit();
	*interrupt_bit=read_bit();
	*crc_bit=read_bit();
	feed_phase(10);
	//return alarm_bit && interrupt_bit && crc_bit;
}



//функция отключения от общения устройства
void off(uint8_t * alarm_bit,uint8_t * interrupt_bit,uint8_t *crc_bit){
	//сегмент функциональной команды
	uint8_t crc=crc5(0x01);
	send_segment(0x01, crc);
	*alarm_bit=read_bit();
	*interrupt_bit=read_bit();
	*crc_bit=read_bit();
	feed_phase(10);
	//return alarm_bit && interrupt_bit && crc_bit;
}

//функция подключения к общению устройства
void on(uint8_t * alarm_bit,uint8_t * interrupt_bit,uint8_t *crc_bit){
	//сегмент функциональной команды
	uint8_t crc=crc5(0x02);
	send_segment(0x02, crc);
	*alarm_bit=read_bit();
	*interrupt_bit=read_bit();
	*crc_bit=read_bit();
	feed_phase(10);
	//return alarm_bit && interrupt_bit && crc_bit;
}

// Основная функция отправки пакета
// ROM_func: код ROM-команды (0xCC, 0x55 и т.д.)
// func_command: код функциональной команды (0x01, 0x02 ...)
// datasize: размер массива данных (пока не используется, но можно добавить)
// data: массив данных (пока не используется)
void send_pack(uint8_t ROM_func, uint8_t func_command, uint8_t datasize, uint8_t data[]){
	// Локальные переменные для хранения битов ответа
	uint8_t alarm_bit = 0, interrupt_bit = 0, crc_bit = 0;
	// Отправляем ROM-команду (например, 0xCC – SKIP ROM)
	switch (ROM_func){
		case 0xCC:
			do{
				ROM_skip(&alarm_bit, &interrupt_bit, &crc_bit);
			}while(!crc_bit);
			break;
		default:
			break;
	}
	//далее у всех функций сначала отправляется размер данных (функциональная команда+данные) именно количество сегментов, потом фанкциональная команда, затем данные если есть
	//отправляем размер данных
	do{
		send_segment(datasize+1, crc5(datasize+1));
		alarm_bit=read_bit();
		interrupt_bit=read_bit();
		crc_bit=read_bit();
		feed_phase(10);
	}while(!crc_bit);

	// После ROM-команды отправляем функциональную команду
	switch (func_command){
		case 0x01:
			do{
				off(&alarm_bit, &interrupt_bit, &crc_bit);
			}while(!crc_bit);
			break;
		case 0x02:
			do{
				on(&alarm_bit, &interrupt_bit, &crc_bit);
			}while(!crc_bit);
			break;
		default:
			break;
	}

    // Если есть данные (datasize > 0), их нужно отправить сегментами
    // Пока не реализовано, будет в других функциях
    if (datasize > 0) {
        // Например, отправить каждый байт данных отдельным сегментом
        for (uint8_t i = 0; i < datasize; i++) {
            do {
                uint8_t crc = crc5(data[i]);
                send_segment(data[i], crc);
                alarm_bit = read_bit();
                interrupt_bit = read_bit();
                crc_bit = read_bit();
                feed_phase(10);
            } while (!crc_bit);
        }
    }
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
  /* USER CODE BEGIN 2 */
  DWT_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
