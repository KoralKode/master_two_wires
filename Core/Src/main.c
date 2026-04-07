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
#define TIMESLOT_US     70
#define CRC5_POLY       0x05
#define ROM_SKIP_CMD    0xCC
#define FUNC_OFF        0x01
#define FUNC_ON         0x02
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ========== НАСТРОЙКА ТАЙМЕРА (TIM2) ==========
// Предполагается, что таймер TIM2 уже настроен в CubeMX:
//   Prescaler = SystemCoreClock / 1_000_000 - 1   (тик 1 мкс)
//   Period = TIMESLOT_US - 1                      (70-1 = 69)
//   AutoReload Preload = Enable
//   NVIC: TIM2 global interrupt enabled

// ========== Глобальные переменные для автомата ==========
typedef enum {
    IDLE,                      // нет активной операции
    // состояния для одного сегмента (отправка + приём трёх битов)
    SEG_START_BIT,             // отправка стартового бита (0)
    SEG_DATA_BITS,             // отправка 8 бит данных
    SEG_CRC_BITS,              // отправка 5 бит CRC
    SEG_RECV_ALARM,            // приём бита аварии (первый из трёх)
    SEG_RECV_INTR,             // приём бита прерывания (второй)
    SEG_RECV_CRC,              // приём бита подтверждения CRC (третий)
    SEG_COMPLETE,              // сегмент завершён, обработка результата
    // состояния для высокоуровневой последовательности (пакет)
    SEQ_ROM,                   // отправка ROM-команды
    SEQ_POWER_AFTER_ROM,       // фаза питания после ROM-команды
    SEQ_SIZE,                  // отправка сегмента с размером данных
    SEQ_POWER_AFTER_SIZE,      // фаза питания после размера
    SEQ_FUNC,                  // отправка функциональной команды
    SEQ_POWER_AFTER_FUNC,      // фаза питания после функциональной команды
    SEQ_DATA,                  // отправка байтов данных (один или несколько)
	SEQ_WAIT_PHASE,     // ожидание окончания фазы питания
    SEQ_END                    // пакет завершён
} state_t;

static void next_step(void);   // прототип функции для отправки пакета
static state_t current_seq_state = IDLE;     // сохранённый этап последовательности (SEQ_*)
static state_t global_state = IDLE;   // текущее состояние автомата
static uint8_t tx_data = 0;           // данные для отправки в текущем сегменте
static uint8_t tx_crc = 0;            // CRC для текущего сегмента
static uint8_t tx_bit_index = 0;      // индекс текущего бита при отправке (0..7 или 0..4)
static uint8_t rx_flags = 0;          // биты: 0=alarm, 1=interrupt, 2=crc_bit

// Параметры пакета (из вызова send_pack)
static uint8_t current_rom_cmd = 0;        // ROM-команда текущего пакета
static uint8_t current_func_cmd = 0;       // функциональная команда текущего пакета
static uint8_t current_datasize = 0;       // количество байт данных в пакете
static uint8_t *current_data = NULL;       // указатель на массив данных
static uint8_t data_index = 0;             // индекс текущего байта данных при отправке
// Переменные для неблокирующей фазы питания
static uint16_t phase_ticks = 0;            // сколько тиков (70 мкс) осталось ждать
static state_t next_state_after_phase = IDLE; // состояние после фазы

// Флаги для циклов подтверждения (как do-while)
static uint8_t retry_needed = 0;           // =1 если сегмент нужно повторить (ошибка CRC)

// ========== Работа с линией PB0 ==========
// Макросы для быстрого управления выводом PB0 (открытый сток)
#define DATA_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1)   // отпустить линию (1)
#define DATA_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0) // притянуть к земле (0)
#define DATA_READ()  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)                   // прочитать состояние линии

// ========== Управление таймером ==========
void start_timer(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);   // обнуляем счётчик таймера
    HAL_TIM_Base_Start_IT(&htim2);      // запускаем таймер с прерываниями
}

void stop_timer(void) {
    HAL_TIM_Base_Stop_IT(&htim2);       // останавливаем таймер
}

// ========== CRC-5 (MSB first) ==========
uint8_t crc5(uint8_t data) {
    uint8_t crc = 0x00;                 // начальное значение регистра CRC
    for (int i = 7; i >= 0; i--) {      // проходим по битам от старшего к младшему
        uint8_t bit = (data >> i) & 1;  // извлекаем текущий бит данных
        uint8_t msb = (crc >> 4) & 1;   // старший бит текущего CRC (бит 4)
        crc = ((crc << 1) | bit) & 0x1F; // сдвигаем влево и добавляем бит данных, отсекаем лишнее
        if (msb) crc ^= CRC5_POLY;       // если выдвинулась 1, выполняем XOR с полиномом
    }
    return crc;                         // возвращаем 5-битный CRC
}

// ========== Запуск одного сегмента ==========
void start_segment(uint8_t data, uint8_t crc, state_t seq_state) {
    current_seq_state = seq_state;   // запоминаем этап последовательности
    tx_data = data;
    tx_crc = crc;
    tx_bit_index = 0;
    rx_flags = 0;
    global_state = SEG_START_BIT;
    start_timer();
}



// ========== Колбэк завершения сегмента ==========
static void on_segment_done(void) {
    if (retry_needed) {
        // Повторяем сегмент: используем сохранённый этап
        switch (current_seq_state) {
            case SEQ_ROM:  start_segment(current_rom_cmd, crc5(current_rom_cmd), SEQ_ROM); break;
            case SEQ_SIZE: start_segment(current_datasize + 1, crc5(current_datasize + 1), SEQ_SIZE); break;
            case SEQ_FUNC: start_segment(current_func_cmd, crc5(current_func_cmd), SEQ_FUNC); break;
            case SEQ_DATA: start_segment(current_data[data_index], crc5(current_data[data_index]), SEQ_DATA); break;
            default: break;
        }
    } else {
        // Успешно – переходим к следующему этапу последовательности
        switch (current_seq_state) {
        case SEQ_ROM:  global_state = SEQ_POWER_AFTER_ROM; next_step(); break;
        case SEQ_SIZE: global_state = SEQ_POWER_AFTER_SIZE; next_step(); break;
        case SEQ_FUNC: global_state = SEQ_POWER_AFTER_FUNC; next_step(); break;
        case SEQ_DATA: global_state = SEQ_DATA; next_step(); break;
        default: break;
        }
    }
}

// ========== Единственный обработчик прерывания таймера ==========
// Вызывается каждые TIMESLOT_US микросекунд
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM2) return;   // проверяем, что прерывание от нужного таймера

    switch (global_state) {
        // --- Отправка сегмента ---
        case SEG_START_BIT:
            DATA_LOW();                   // выставляем 0 на весь (почти) слот
            global_state = SEG_DATA_BITS; // переходим к отправке данных
            break;

        case SEG_DATA_BITS:
            if (tx_bit_index < 8) {                       // пока не отправили 8 бит
                uint8_t bit = (tx_data >> (7 - tx_bit_index)) & 1; // берём бит от старшего
                if (bit) DATA_HIGH(); else DATA_LOW();    // выставляем бит на линию

                if(tx_bit_index ==7){
                	tx_bit_index = 0;                         // сброс индекса
                    global_state = SEG_CRC_BITS;              // переходим к отправке CRC
                    break;
                }
                tx_bit_index++;

            }
            break;

        case SEG_CRC_BITS:
            if (tx_bit_index < 5) {                       // пока не отправили 5 бит CRC
                uint8_t bit = (tx_crc >> (4 - tx_bit_index)) & 1; // берём бит CRC от старшего
                if (bit) DATA_HIGH(); else DATA_LOW();

                if(tx_bit_index ==4){
                    //DATA_HIGH();                              // отпускаем линию после CRC
                    global_state = SEG_RECV_ALARM;            // переходим к приёму первого ответного бита
                    break;
                }
                tx_bit_index++;
            }
            break;

        // --- Приём трёх битов ответа ---
        case SEG_RECV_ALARM:
        	DATA_HIGH();
            if (DATA_READ()) rx_flags |= 0x01;   // если линия = 1, устанавливаем бит аварии
            global_state = SEG_RECV_INTR;        // следующий бит
            break;

        case SEG_RECV_INTR:
            if (DATA_READ()) rx_flags |= 0x02;   // бит прерывания
            global_state = SEG_RECV_CRC;
            break;

        case SEG_RECV_CRC:
            if (DATA_READ()) rx_flags |= 0x04;   // бит подтверждения CRC
            stop_timer();                                   // останавливаем таймер
            retry_needed = ((rx_flags & 0x04) == 0) ? 1 : 0; // если crc_bit = 0, нужно повторить
            global_state = IDLE;                            // временно в неактивное состояние
            on_segment_done();                              // вызываем обработчик завершения
            break;

            // --- Ожидание фазы питания (неблокирующее) ---
        case SEQ_WAIT_PHASE:
                if (phase_ticks > 0) {
                    phase_ticks--;
                } else {
                    global_state = next_state_after_phase;
                    next_step();      // продолжаем выполнение пакета
                }
                break;

        default:
            break;  // для других состояний (SEQ_*) ничего не делаем в прерывании
    }
}

// ========== Определение next_step (перенесено выше) ==========
static void next_step(void) {
    uint8_t crc;
    //uint16_t delay_ticks;
    switch (global_state) {
    case SEQ_ROM:
         crc = crc5(current_rom_cmd);
         start_segment(current_rom_cmd, crc, SEQ_ROM);// отправляем rom команду
         break;
    case SEQ_POWER_AFTER_ROM:
        // 10 мс = 10 000 мкс / 70 ≈ 143 тика
        phase_ticks = (10 * 1000) / TIMESLOT_US;
        next_state_after_phase = SEQ_SIZE;
        global_state = SEQ_WAIT_PHASE;
        start_timer();
        break;
        case SEQ_SIZE:
            crc = crc5(current_datasize + 1);            // CRC для байта размера (данные+функц. команда)
            start_segment(current_datasize + 1, crc,SEQ_SIZE);    // отправляем размер
            break;
        case SEQ_POWER_AFTER_SIZE:
            phase_ticks = (10 * 1000) / TIMESLOT_US;
            next_state_after_phase = SEQ_FUNC;
            global_state = SEQ_WAIT_PHASE;
            start_timer();
            break;


        case SEQ_FUNC:
            crc = crc5(current_func_cmd);                // CRC для функциональной команды
            start_segment(current_func_cmd, crc, SEQ_FUNC);        // отправляем
            break;
        case SEQ_POWER_AFTER_FUNC:
            phase_ticks = (10 * 1000) / TIMESLOT_US;
            if (current_datasize > 0) {
                next_state_after_phase = SEQ_DATA;
            } else {
                next_state_after_phase = SEQ_END;
            }
            global_state = SEQ_WAIT_PHASE;
            start_timer();
            break;


        case SEQ_DATA:
            if (data_index < current_datasize) {
                crc = crc5(current_data[data_index]);     // CRC для очередного байта данных
                start_segment(current_data[data_index], crc,SEQ_DATA); // отправляем сегмент
                data_index++;                             // увеличиваем индекс
            } else {
                global_state = SEQ_END;                   // все данные отправлены
                next_step();
            }
            break;

        case SEQ_END:
            global_state = IDLE;                          // возвращаемся в бездействие
            stop_timer();
            break;

        default:
            break;
    }
}

// ========== Функция send_pack_async (вызывает next_step) ==========
void send_pack_async(uint8_t rom_cmd, uint8_t func_cmd, uint8_t datasize, uint8_t *data) {
    current_rom_cmd = rom_cmd;          // запоминаем ROM-команду
    current_func_cmd = func_cmd;        // запоминаем функциональную команду
    current_datasize = datasize;        // запоминаем размер данных
    current_data = data;                // запоминаем указатель на данные
    data_index = 0;                     // начинаем с первого байта

    global_state = SEQ_ROM;             // устанавливаем начальное состояние
    next_step();                        // запускаем последовательность
}

// ========== Инициализация протокола ==========
void protocol_init(void) {
    DATA_HIGH();                        // устанавливаем линию в 1 (высокий уровень, резистор подтянет)
    // Таймер уже настроен в CubeMX, прерывания разрешены
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(TIM2_IRQn);
  protocol_init();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      //HAL_Delay(500);
	  int i=0;
	  while(i<1000000){
		  ++i;
	  }
	  i=0;
      // Запускаем пакет асинхронно (без блокировки)
      send_pack_async(ROM_SKIP_CMD, FUNC_OFF, 0, NULL);
      //HAL_Delay(500);
	  while(i<1000000){
		  ++i;
	  }
	  i=0;
      send_pack_async(ROM_SKIP_CMD, FUNC_ON, 0, NULL);

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 69;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
