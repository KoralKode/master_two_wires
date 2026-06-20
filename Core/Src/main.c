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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "protocol.h"
#include "comport.h"
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


// ========== Единственный обработчик прерывания таймера 2 ==========
// Вызывается каждые TIMESLOT_US / 2 микросекунд.
// Один логический тайм-слот протокола состоит из двух полутактов таймера.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	protocol_timer_irq_handler(htim);
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  //настройка прерываний от таймера
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  protocol_init();//инициализация протокола
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  //search_rom_packet_blocking();
  // Тип последнего запущенного пакета нужен только для вывода результата
  static uint8_t active_packet_type = 0xFFU;
  static uint8_t protocol_was_busy = 0U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	    // 1. Если протокол только что завершился – вывести результат
	    if (protocol_was_busy && !protocol_is_busy())
	    {
	        protocol_was_busy = 0U;

	        if (protocol_packet_aborted_by_alarm)
	        {
	            // Пакет был остановлен из-за alarm.
	            // Поэтому не пишем "TX done" или "RX done",
	            // чтобы не создавать ложное впечатление успешного завершения.
	            comport_send_response("PACKET stopped: alarm detected\r\n");
	            protocol_clear_packet_abort_flag();
	        }
	        else if (active_packet_type == PACKET_TYPE_SEARCH)
	        {
	            comport_send_response("SEARCH done\r\n");
	            comport_print_found_roms();

	            /*
	             * После обычного поиска адресов запускаем получение параметров питания.
	             *
	             * Здесь параметры НЕ передаются на ПК.
	             * Мастер только поочерёдно отправляет каждому найденному ведомому
	             * пакет MATCH + PARAMETERS, принимает 10 байт ответа
	             * и сохраняет их во внутренний массив power_params[].
	             */
	            (void)power_params_request_all_async();
	        }
	        else if (active_packet_type == PACKET_TYPE_RX)
	        {
	            comport_send_response("RX done\r\n");
	            comport_print_received_data();
	        }
	        else if (active_packet_type == PACKET_TYPE_TX)
	        {
	            comport_send_response("TX done\r\n");
	        }

	        active_packet_type = 0xFFU;
	    }
		  // 1.1. Фоновая обработка внутренних событий протокола.
		  // Здесь обрабатывается бит прерывания, принятый в завершённом пакете.
		  //
		  // Важно: protocol_poll() ничего не делает, пока протокол занят.
	    // Поэтому обработка прерывания начнётся только после завершения
	    // текущего пользовательского пакета.
	    protocol_poll();
	    // 2. Пока протокол занят, COM-порт не обрабатываем,
	    // чтобы USB не мешал таймингу передачи.
	    if (protocol_is_busy())
	    {
	        continue;
	    }

	    // 2.1. Если внутренняя обработка прерывания или alarm завершилась,
	    // выводим результат пользователю через COM-порт.
	    //
	    // Новые устройства добавляются в основной массив found_roms[].
	    // Устройства с прерыванием хранятся в interrupt_roms[].
	    // Устройства с alarm хранятся в alarm_roms[].
	    if (protocol_event_type != PROTOCOL_EVENT_NONE)
	    {
	    	if (protocol_event_type == PROTOCOL_EVENT_NEW_DEVICES)
	    	{
	    	    comport_send_response("INTERRUPT: new device detected\r\n");
	    	    comport_send_response("Updated ROM-search result:\r\n");
	    	    comport_print_found_roms();

	    	    /*
	    	     * После обнаружения новых устройств нужно получить их параметры питания.
	    	     *
	    	     * Функция power_params_request_all_async() проходит по found_roms[]
	    	     * и запрашивает PARAMETERS только у тех адресов,
	    	     * для которых ещё нет актуальной записи в power_params[].
	    	     *
	    	     * Поэтому старые ведомые повторно не опрашиваются,
	    	     * а новое ведомое, добавленное в конец found_roms[],
	    	     * получит параметры в такой же ячейке массива power_params[].
	    	     */
	    	    (void)power_params_request_all_async();
	    	}
	        else if (protocol_event_type == PROTOCOL_EVENT_INTERRUPT_DEVICES)
	        {
	            comport_send_response("INTERRUPT: devices with interrupt flag\r\n");
	            comport_send_response("Interrupt devices: %u\r\n", interrupt_rom_count);

	            // Печатаем адреса из отдельного массива interrupt_roms.
	            // Основной список found_roms при этом не изменяется.
	            for (uint8_t i = 0U; i < interrupt_rom_count; i++)
	            {
	                comport_send_response("INT_ROM[%u]: %02X %02X %02X %02X\r\n",
	                                      i,
	                                      interrupt_roms[i][0],
	                                      interrupt_roms[i][1],
	                                      interrupt_roms[i][2],
	                                      interrupt_roms[i][3]);
	            }
	        }else if (protocol_event_type == PROTOCOL_EVENT_ALARM_DEVICES)
	        {
	            comport_send_response("ALARM: devices with alarm flag\r\n");
	            comport_send_response("Alarm devices: %u\r\n", alarm_rom_count);

	            // Печатаем адреса из отдельного массива alarm_roms.
	            // Основной список found_roms при этом не изменяется.
	            for (uint8_t i = 0U; i < alarm_rom_count; i++)
	            {
	                comport_send_response("ALARM_ROM[%u]: %02X %02X %02X %02X\r\n",
	                                      i,
	                                      alarm_roms[i][0],
	                                      alarm_roms[i][1],
	                                      alarm_roms[i][2],
	                                      alarm_roms[i][3]);
	            }
	        }else if (protocol_event_type == PROTOCOL_EVENT_POWER_PARAMS_READY)
	        {
	            /*
	             * Все RX-пакеты PARAMETERS завершены.
	             * Параметры уже сохранены в power_params[],
	             * теперь их можно вывести на ПК.
	             */
	            comport_print_power_params();
	        }else if (protocol_event_type == PROTOCOL_EVENT_FEED_PLAN_SENT)
	        {
	            /*
	             * Все индивидуальные пакеты FEED_PLAN отправлены ведомым.
	             * На этом этапе план только передан ведомым,
	             * выполнение цикла питания добавим отдельно.
	             */
	            comport_send_response("FEED PLAN sent to devices\r\n");
	        }

	        // После вывода события разрешаем протоколу принимать новые события.
	        protocol_clear_event();
	        /*
	         * Обработка события могла запустить внутренний пакет протокола,
	         * например MATCH + PARAMETERS после обнаружения нового ведомого.
	         * В таком случае COM-порт в этом проходе цикла не обрабатываем,
	         * чтобы не попытаться запустить пользовательскую команду
	         * параллельно с внутренним обменом.
	         */
	        if (protocol_is_busy())
	        {
	            continue;
	        }
	    }

	    // 3. Протокол свободен – можно читать команды из COM-порта
	    comport_poll();

	    // 4. Если из COM-порта подготовлен пакет – запускаем его
	    if (comport_command_ready)
	    {
	        bool started = false;

	        if (packet_type == PACKET_TYPE_TX)
	        {
	            uint8_t *addr_ptr = NULL;
	            uint8_t *data_ptr = NULL;

	            if (rom_cmd == ROM_MATCH_CMD)
	            {
	                addr_ptr = address_arr;
	            }

	            if (data_size > 0U)
	            {
	                data_ptr = data_arr;
	            }

	            started = send_pack_async(rom_cmd,
	                                      func_cmd,
	                                      data_size,
	                                      data_ptr,
	                                      addr_ptr);
	        }
	        else if (packet_type == PACKET_TYPE_RX)
	        {
	            started = receive_pack_async(func_cmd,
	                                         data_size,
	                                         address_arr);
	        }
	        else if (packet_type == PACKET_TYPE_SEARCH)
	        {
	            started = search_rom_packet_async();
	        }

	        if (started)
	        {
	            active_packet_type = packet_type;
	            protocol_was_busy = 1U;
	            comport_clear_command();
	        }
	        else
	        {
	            comport_send_response("Error: protocol did not start\r\n");
	        }
	    }


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  htim2.Init.Period = 34;
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
